async function createGraspViewer(id) {
  const THREE = await import('three');
  const element = getElement(id);
  for (let i = 0; !element.is_initialized && i < 300; i++) {
    await new Promise(resolve => setTimeout(resolve, 50));
  }
  if (!element.renderer) throw new Error('WebGL is unavailable in this browser');
  const scene = element.scene;
  const content = new THREE.Group();
  scene.add(content);
  let data = null;
  let selected = 0;
  let displayMode = 'all';
  let groups = [];
  let objectGroup = new THREE.Group();
  let grid = new THREE.Group();
  let axes = new THREE.AxesHelper(1);
  let opacity = 0.3;
  let colorMode = 'direction';
  let center = new THREE.Vector3();
  let radius = 1;
  const highlight = new THREE.Color('#d69639');
  const loader = new THREE.TextureLoader();
  element.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  const observer = new ResizeObserver(() => element.resize());
  observer.observe(element.$el);

  function geometry(mesh) {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(mesh.vertices, 3));
    geometry.setIndex(mesh.faces);
    if (mesh.uv) geometry.setAttribute('uv', new THREE.Float32BufferAttribute(mesh.uv, 2));
    geometry.computeVertexNormals();
    return geometry;
  }

  function directionColor(index) {
    if (colorMode === 'uniform') return new THREE.Color('#419b90');
    const axis = data.candidates[index].approach_axis_object;
    const c = new THREE.Color(0, 0, 0);
    const colors = ['#e29448', '#42b6a0', '#7785cc'];
    const total = axis.reduce((sum, v) => sum + Math.abs(v), 0) || 1;
    axis.forEach((v, i) => c.add(new THREE.Color(colors[i]).multiplyScalar(Math.abs(v) / total)));
    return c;
  }

  function refresh() {
    if (!data) return;
    for (const group of groups) {
      group.all.visible = displayMode === 'all';
      group.all.material.opacity = opacity;
      for (let i = 0; i < data.candidates.length; i++) {
        group.all.setColorAt(i, directionColor(i));
        const matrix = new THREE.Matrix4().fromArray(group.matrices[i]);
        // Selected geometry is rendered once, opaque, above the distribution.
        if (i === selected) matrix.scale(new THREE.Vector3(0, 0, 0));
        group.all.setMatrixAt(i, matrix);
      }
      group.all.instanceMatrix.needsUpdate = true;
      group.all.instanceColor.needsUpdate = true;
      group.single.matrix.fromArray(group.matrices[selected]);
    }
    const pose = data.candidates[selected].pose_object_tcp_xyz_xyzw;
    axes.position.fromArray(pose);
    axes.quaternion.fromArray(pose.slice(3));
  }

  function dispose() {
    const geometries = new Set(), materials = new Set(), textures = new Set();
    content.traverse(o => {
      if (o.geometry) geometries.add(o.geometry);
      for (const m of Array.isArray(o.material) ? o.material : o.material ? [o.material] : []) {
        materials.add(m);
        if (m.map) textures.add(m.map);
      }
      if (o.isInstancedMesh) o.dispose();
    });
    geometries.forEach(g => g.dispose());
    materials.forEach(m => m.dispose());
    textures.forEach(t => t.dispose());
    content.clear();
    groups = [];
  }

  const api = {
    async load(index) {
      const response = await fetch(`/grasp-data/${index}`);
      const next = await response.json();
      if (!response.ok) throw new Error(next.detail || 'Dataset loading failed');
      dispose();
      data = next;
      selected = 0;
      objectGroup = new THREE.Group();
      content.add(objectGroup);
      for (const mesh of data.object_meshes) {
        const material = new THREE.MeshStandardMaterial({
          color: new THREE.Color().fromArray(mesh.color), roughness: 0.66, metalness: 0.05,
          side: THREE.DoubleSide,
        });
        if (mesh.texture) {
          material.map = await loader.loadAsync(mesh.texture);
          material.map.colorSpace = THREE.SRGBColorSpace;
          material.color.set('#ffffff');
        }
        objectGroup.add(new THREE.Mesh(geometry(mesh), material));
      }
      for (const part of data.parts) {
        for (const mesh of part.meshes) {
          const g = geometry(mesh);
          const overview = mesh.overview ? geometry(mesh.overview) : g;
          const all = new THREE.InstancedMesh(overview, new THREE.MeshStandardMaterial({
            color: '#ffffff', transparent: true, opacity, depthWrite: false,
            roughness: 0.6, metalness: 0.15, side: THREE.FrontSide,
          }), data.candidates.length);
          all.frustumCulled = false;
          for (let i = 0; i < data.candidates.length; i++) {
            all.setMatrixAt(i, new THREE.Matrix4().fromArray(part.matrices[i]));
          }
          all.computeBoundingSphere();
          const single = new THREE.Mesh(g, new THREE.MeshStandardMaterial({
            color: highlight, roughness: 0.45, metalness: 0.25, side: THREE.DoubleSide,
          }));
          single.matrixAutoUpdate = false;
          content.add(all, single);
          groups.push({all, single, matrices: part.matrices});
        }
      }
      const bounds = new THREE.Box3().setFromObject(content);
      center = bounds.getCenter(new THREE.Vector3());
      radius = Math.max(bounds.getSize(new THREE.Vector3()).length() / 2, 0.001);
      const min = new THREE.Vector3().fromArray(data.bounds[0]);
      const max = new THREE.Vector3().fromArray(data.bounds[1]);
      const objectSize = max.clone().sub(min);
      grid = new THREE.GridHelper(radius * 4, 24, '#b6c4be', '#d4dcd8');
      grid.rotation.x = Math.PI / 2;
      grid.position.set(center.x, center.y, min.z - objectSize.length() * 0.03);
      grid.material.transparent = true;
      grid.material.opacity = 0.6;
      axes = new THREE.AxesHelper(objectSize.length() * 0.25);
      axes.visible = false;
      content.add(grid, axes);
      refresh();
      api.frame('perspective');
      return {object: data.object, robot: data.robot, candidates: data.candidates,
        provenance: data.provenance, source: data.source, size: objectSize.toArray()};
    },
    select(index) { selected = index; refresh(); },
    mode(value) { displayMode = value; refresh(); },
    appearance(value, showObject, showAxes, showGrid, wireframe, color) {
      opacity = value;
      colorMode = color;
      objectGroup.visible = showObject;
      axes.visible = showAxes;
      grid.visible = showGrid;
      for (const group of groups) {
        group.all.material.wireframe = wireframe;
        group.single.material.wireframe = wireframe;
      }
      refresh();
    },
    frame(view) {
      if (!data) return;
      content.updateMatrixWorld(true);
      const bounds = new THREE.Box3().setFromObject(objectGroup);
      for (const group of groups) {
        bounds.expandByObject(displayMode === 'all' ? group.all : group.single);
      }
      center = bounds.getCenter(new THREE.Vector3());
      const direction = view === 'top' ? new THREE.Vector3(0, -0.001, 1)
        : view === 'front' ? new THREE.Vector3(0, -1, 0) : new THREE.Vector3(1, -1.6, 0.95);
      direction.normalize();
      const camera = element.camera;
      const fov = THREE.MathUtils.degToRad(camera.fov);
      const right = new THREE.Vector3().crossVectors(new THREE.Vector3(0, 0, 1), direction).normalize();
      const up = new THREE.Vector3().crossVectors(direction, right).normalize();
      let distance = 0;
      for (const x of [bounds.min.x, bounds.max.x]) {
        for (const y of [bounds.min.y, bounds.max.y]) {
          for (const z of [bounds.min.z, bounds.max.z]) {
            const corner = new THREE.Vector3(x, y, z).sub(center);
            distance = Math.max(distance,
              Math.abs(corner.dot(right)) / (Math.tan(fov / 2) * camera.aspect) + corner.dot(direction),
              Math.abs(corner.dot(up)) / Math.tan(fov / 2) + corner.dot(direction));
          }
        }
      }
      camera.near = radius / 1000;
      camera.far = radius * 1000;
      camera.position.copy(center).add(direction.multiplyScalar(distance * 1.12));
      camera.up.set(0, 0, 1);
      element.controls.target.copy(center);
      element.controls.minDistance = radius * 0.05;
      element.controls.maxDistance = radius * 50;
      camera.lookAt(center);
      camera.updateProjectionMatrix();
      element.controls.update();
    },
    snapshot() {
      element.renderer.render(scene, element.camera);
      const link = document.createElement('a');
      link.download = `${data ? data.object : 'scene'}-${displayMode}-${selected}.png`;
      link.href = element.renderer.domElement.toDataURL('image/png');
      link.click();
    },
    inspect() {
      return {count: data?.candidates.length || 0, selected, mode: displayMode,
        instances: groups.reduce((n, g) => n + (g.all.visible ? g.all.count : 0), 0),
        camera: element.camera.position.toArray(), meshes: objectGroup.children.length,
        textures: objectGroup.children.filter(m => m.material.map).length};
    },
  };
  let down = {x: 0, y: 0};
  element.renderer.domElement.addEventListener('pointerdown', e => { down = {x: e.clientX, y: e.clientY}; });
  element.renderer.domElement.addEventListener('pointerup', event => {
    if (!data || displayMode !== 'all' || Math.hypot(event.clientX - down.x, event.clientY - down.y) > 4) return;
    const rect = element.renderer.domElement.getBoundingClientRect();
    const pointer = new THREE.Vector2((event.clientX - rect.left) / rect.width * 2 - 1,
      -(event.clientY - rect.top) / rect.height * 2 + 1);
    const ray = new THREE.Raycaster();
    ray.setFromCamera(pointer, element.camera);
    const hits = ray.intersectObjects(groups.map(g => g.all));
    if (hits.length && hits[0].instanceId !== undefined) element.$emit('grasp_pick', hits[0].instanceId);
  });
  return api;
}
