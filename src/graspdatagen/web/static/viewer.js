async function createGraspViewer(id, timeoutMs) {
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
  let datasetIndex = 0;
  let loadController = new AbortController();
  let displayMode = 'all';
  let groups = [];
  let objectGroup = new THREE.Group();
  let grid = new THREE.Group();
  let axes = new THREE.AxesHelper(1);
  let opacity = 0.3;
  let colorMode = 'direction';
  let workspaceMode = 'preview';
  let annotationMesh = null;
  let annotationFaces = new Set();
  let annotationMode = 'paint';
  let annotationBrushPercent = 3;
  let annotationAdjacency = [];
  let annotationFaceCenters = new Float32Array();
  let annotationObjectRadius = 1;
  let annotationHistory = [];
  let painting = false;
  let activePointerId = null;
  let strokeSnapshot = null;
  let previousPaintFace = null;
  let center = new THREE.Vector3();
  let radius = 1;
  const highlight = new THREE.Color('#d69639');
  const loader = new THREE.TextureLoader();
  element.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  const defaultMouseButtons = {...element.controls.mouseButtons};
  const observer = new ResizeObserver(() => element.resize());
  observer.observe(element.$el);

  function geometry(mesh) {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(mesh.vertices, 3));
    geometry.setIndex(new THREE.BufferAttribute(mesh.faces, 1));
    if (mesh.uv) geometry.setAttribute('uv', new THREE.BufferAttribute(mesh.uv, 2));
    geometry.computeVertexNormals();
    return geometry;
  }

  function refreshAnnotation() {
    if (!annotationMesh) return;
    const colors = annotationMesh.geometry.getAttribute('color');
    const base = new THREE.Color('#8b9599');
    const selectedColor = new THREE.Color('#e06a42');

    for (let face = 0; face < colors.count / 3; face++) {
      const color = annotationFaces.has(face) ? selectedColor : base;
      for (let corner = 0; corner < 3; corner++) {
        colors.setXYZ(
          face * 3 + corner,
          color.r,
          color.g,
          color.b,
        );
      }
    }
    colors.needsUpdate = true;
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
      group.all.visible = workspaceMode === 'preview' && displayMode === 'all';
      group.single.visible = workspaceMode === 'preview';
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
    if (!data.candidates.length) {
      axes.visible = false;
      return;
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
    annotationMesh = null;
    annotationFaces = new Set();
    annotationAdjacency = [];
    annotationFaceCenters = new Float32Array();
    annotationObjectRadius = 1;
    annotationHistory = [];
  }

  async function progress(message) {
    element.$emit('load_progress', message);
    // Give the browser a chance to paint the progress indicator between stages.
    await new Promise(resolve => setTimeout(resolve, 0));
  }

  function loadTexture(url, signal) {
    return new Promise((resolve, reject) => {
      const cancel = () => { texture.dispose(); reject(signal.reason); };
      const texture = loader.load(url, () => {
        signal.removeEventListener('abort', cancel);
        resolve(texture);
      }, undefined, () => {
        signal.removeEventListener('abort', cancel);
        texture.dispose();
        reject(new Error('Could not load the object texture'));
      });
      signal.addEventListener('abort', cancel, {once: true});
      if (signal.aborted) cancel();
    });
  }

  async function fetchPayload(url, signal) {
    const response = await fetch(url, {signal});
    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.detail || `Dataset loading failed (${response.status})`);
    }
    const buffer = await response.arrayBuffer();
    signal.throwIfAborted();
    const headerLength = new DataView(buffer).getUint32(0, true);
    const header = new TextDecoder().decode(new Uint8Array(buffer, 4, headerLength));
    const start = 4 + headerLength;
    return JSON.parse(header, (_key, value) => {
      if (value && value.buffer_type) {
        const Type = value.buffer_type === 'uint32' ? Uint32Array : Float32Array;
        return new Type(buffer, start + value.offset, value.length);
      }
      return value;
    });
  }

  function buildAnnotation(mesh) {
    const faceCount = mesh.faces.length / 3;
    const edgeOwners = new Map();
    annotationAdjacency = Array.from(
      {length: faceCount},
      () => new Set(),
    );

    for (let face = 0; face < faceCount; face++) {
      const vertices = [
        mesh.faces[face * 3],
        mesh.faces[face * 3 + 1],
        mesh.faces[face * 3 + 2],
      ];
      for (const [a0, b0] of [
        [vertices[0], vertices[1]],
        [vertices[1], vertices[2]],
        [vertices[2], vertices[0]],
      ]) {
        const a = Math.min(a0, b0);
        const b = Math.max(a0, b0);
        const key = `${a}:${b}`;
        const owners = edgeOwners.get(key) || [];
        for (const other of owners) {
          annotationAdjacency[face].add(other);
          annotationAdjacency[other].add(face);
        }
        owners.push(face);
        edgeOwners.set(key, owners);
      }
    }

    const vertices = mesh.vertices;
    const faces = mesh.faces;
    annotationFaceCenters = new Float32Array(faceCount * 3);

    for (let face = 0; face < faceCount; face++) {
      for (let axis = 0; axis < 3; axis++) {
        annotationFaceCenters[face * 3 + axis] = (
          vertices[faces[face * 3] * 3 + axis]
          + vertices[faces[face * 3 + 1] * 3 + axis]
          + vertices[faces[face * 3 + 2] * 3 + axis]
        ) / 3;
      }
    }

    const indexed = geometry(mesh);
    indexed.computeBoundingSphere();
    annotationObjectRadius = Math.max(
      indexed.boundingSphere?.radius || 0,
      1e-6,
    );
    const annotationGeometry = indexed.toNonIndexed();
    indexed.dispose();

    const count = annotationGeometry.getAttribute('position').count;
    annotationGeometry.setAttribute(
      'color',
      new THREE.Float32BufferAttribute(
        new Float32Array(count * 3),
        3,
      ),
    );

    annotationMesh = new THREE.Mesh(
      annotationGeometry,
      new THREE.MeshStandardMaterial({
        vertexColors: true,
        roughness: 0.7,
        metalness: 0.0,
        side: THREE.DoubleSide,
      }),
    );
    annotationMesh.visible = false;
    content.add(annotationMesh);
    refreshAnnotation();
  }

  function faceCenter(face) {
    return new THREE.Vector3(
      annotationFaceCenters[face * 3],
      annotationFaceCenters[face * 3 + 1],
      annotationFaceCenters[face * 3 + 2],
    );
  }

  function brushRadius() {
    return annotationObjectRadius * annotationBrushPercent / 100;
  }

  function brushFaces(seed, point) {
    const radius = brushRadius();
    const radiusSquared = radius * radius;
    const result = new Set([seed]);
    const queue = [seed];

    for (let cursor = 0; cursor < queue.length; cursor++) {
      const face = queue[cursor];
      for (const neighbour of annotationAdjacency[face] || []) {
        if (result.has(neighbour)) continue;

        const offset = faceCenter(neighbour).sub(point);
        if (offset.lengthSq() <= radiusSquared) {
          result.add(neighbour);
          queue.push(neighbour);
        }
      }
    }
    return result;
  }

  function shortestFacePath(start, goal) {
    if (start === null || start === goal) return [goal];

    const queue = [start];
    const parent = new Map([[start, -1]]);

    for (let cursor = 0; cursor < queue.length; cursor++) {
      const face = queue[cursor];
      for (const neighbour of annotationAdjacency[face] || []) {
        if (parent.has(neighbour)) continue;
        parent.set(neighbour, face);

        if (neighbour === goal) {
          const path = [goal];
          let current = face;
          while (current !== -1) {
            path.push(current);
            current = parent.get(current);
          }
          path.reverse();
          return path;
        }
        queue.push(neighbour);
      }

      // A pointer jump should remain local. Avoid traversing an entire huge mesh
      // when the cursor left and re-entered on an unrelated component.
      if (queue.length > 20000) break;
    }
    return [goal];
  }

  function annotationHitAt(event) {
    if (!annotationMesh) return null;
    const rect = element.renderer.domElement.getBoundingClientRect();
    const pointer = new THREE.Vector2(
      (event.clientX - rect.left) / rect.width * 2 - 1,
      -(event.clientY - rect.top) / rect.height * 2 + 1,
    );
    const ray = new THREE.Raycaster();
    ray.setFromCamera(pointer, element.camera);
    const hits = ray.intersectObject(annotationMesh, false);
    if (!hits.length || hits[0].faceIndex === undefined) return null;

    return {
      face: hits[0].faceIndex,
      point: annotationMesh.worldToLocal(hits[0].point.clone()),
    };
  }

  function applyBrush(seed, point, erase) {
    for (const selectedFace of brushFaces(seed, point)) {
      if (erase) annotationFaces.delete(selectedFace);
      else annotationFaces.add(selectedFace);
    }
  }

  function paintAnnotation(event) {
    const hit = annotationHitAt(event);
    if (!hit) {
      previousPaintFace = null;
      return;
    }

    const erase = annotationMode === 'erase' || event.shiftKey;
    const path = shortestFacePath(previousPaintFace, hit.face);

    // Fill the full connected surface path between pointer events. This removes
    // holes when a dense mesh receives fewer pointermove events than triangles.
    for (const face of path) {
      applyBrush(face, faceCenter(face), erase);
    }

    // Centre the final brush exactly on the ray hit, not merely on the triangle
    // centroid, so the visual brush remains stable on large triangles.
    applyBrush(hit.face, hit.point, erase);
    previousPaintFace = hit.face;
    refreshAnnotation();
  }

  function beginAnnotationStroke(event) {
    if (
      workspaceMode !== 'annotate'
      || annotationMode === 'orbit'
      || !annotationMesh
      || event.button !== 0
    ) return false;

    painting = true;
    activePointerId = event.pointerId;
    strokeSnapshot = new Set(annotationFaces);
    previousPaintFace = null;
    element.controls.enabled = false;
    element.renderer.domElement.setPointerCapture(event.pointerId);
    paintAnnotation(event);
    event.preventDefault();
    event.stopPropagation();
    return true;
  }

  function endAnnotationStroke(event) {
    if (!painting || event.pointerId !== activePointerId) return false;

    painting = false;
    if (strokeSnapshot) annotationHistory.push(strokeSnapshot);
    strokeSnapshot = null;
    previousPaintFace = null;

    if (element.renderer.domElement.hasPointerCapture(event.pointerId)) {
      element.renderer.domElement.releasePointerCapture(event.pointerId);
    }

    activePointerId = null;
    element.controls.enabled = true;
    event.preventDefault();
    event.stopPropagation();
    return true;
  }

  const api = {
    async load(index) {
      loadController.abort();
      const controller = new AbortController();
      loadController = controller;
      const timer = setTimeout(() => controller.abort(new Error('Dataset loading timed out. Retry or select another file.')), timeoutMs);
      try {
        await progress('Reading assets and downloading scene');
        const next = await fetchPayload(`/grasp-data/${index}`, controller.signal);
        await progress('Building object geometry');
        controller.signal.throwIfAborted();
        dispose();
        data = next;
        datasetIndex = index;
        selected = 0;
        objectGroup = new THREE.Group();
        content.add(objectGroup);
        for (const mesh of data.object_meshes) {
          const material = new THREE.MeshStandardMaterial({
            color: new THREE.Color().fromArray(mesh.color), roughness: 0.66, metalness: 0.05,
            side: THREE.DoubleSide,
          });
          if (mesh.texture) {
            const texture = await loadTexture(mesh.texture, controller.signal);
            if (controller.signal.aborted) {
              texture.dispose();
              material.dispose();
              controller.signal.throwIfAborted();
            }
            material.map = texture;
            material.map.colorSpace = THREE.SRGBColorSpace;
            material.color.set('#ffffff');
          }
          objectGroup.add(new THREE.Mesh(geometry(mesh), material));
        }

        await progress('Building grasp poses');
        controller.signal.throwIfAborted();
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
        await progress('Fitting camera');
        controller.signal.throwIfAborted();
        refresh();
        api.frame('perspective');
        return {object: data.object, robot: data.robot, candidates: data.candidates,
          provenance: data.provenance, source: data.source, size: objectSize.toArray()};
      } finally {
        clearTimeout(timer);
      }
    },
    cancelLoad() { loadController.abort(); },
    select(index) { selected = index; refresh(); },
    mode(value) { displayMode = value; refresh(); },

    async workspace(value) {
      if (value === 'annotate' && !annotationMesh) {
        if (!data.has_annotation) throw new Error('This dataset has no prepared annotation surface');
        await progress('Loading annotation surface');
        const payload = await fetchPayload(`/grasp-data/${datasetIndex}/annotation`, AbortSignal.timeout(timeoutMs));
        await progress('Building annotation surface');
        buildAnnotation(payload.annotation_mesh);
      }
      workspaceMode = value;
      objectGroup.visible = value === 'preview';
      if (annotationMesh) annotationMesh.visible = value === 'annotate';

      if (value === 'annotate') {
        element.controls.mouseButtons.RIGHT = THREE.MOUSE.ROTATE;
        element.controls.enabled = true;
      } else {
        Object.assign(element.controls.mouseButtons, defaultMouseButtons);
      }
      refresh();
    },

    setAnnotation(faces) {
      annotationFaces = new Set(faces || []);
      refreshAnnotation();
    },

    async saveAnnotation() {
      if (!data || !annotationMesh) {
        throw new Error('No annotation surface is loaded');
      }

      const response = await fetch(
        `/grasp-data/${datasetIndex}/annotation`,
        {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({
            faces: Array.from(annotationFaces).sort((a, b) => a - b),
          }),
        },
      );

      let payload;
      try {
        payload = await response.json();
      } catch (_error) {
        throw new Error(`Save failed with HTTP ${response.status}`);
      }

      if (!response.ok) {
        throw new Error(
          payload.detail || `Save failed with HTTP ${response.status}`
        );
      }
      return payload;
    },

    annotationFaces() {
      return Array.from(annotationFaces).sort((a, b) => a - b);
    },

    annotationTool(value) {
      annotationMode = value;
      painting = false;
      activePointerId = null;
      element.controls.enabled = true;
    },

    annotationBrush(value) {
      annotationBrushPercent = Math.max(
        0.5,
        Math.min(50, Number(value) || 5),
      );
    },

    undoAnnotation() {
      if (!annotationHistory.length) return annotationFaces.size;
      annotationFaces = annotationHistory.pop();
      refreshAnnotation();
      return annotationFaces.size;
    },

    clearAnnotation() {
      annotationHistory.push(new Set(annotationFaces));
      annotationFaces.clear();
      refreshAnnotation();
      return 0;
    },

    appearance(value, showObject, showAxes, showGrid, wireframe, color) {
      opacity = value;
      colorMode = color;
      objectGroup.visible = workspaceMode === 'preview' && showObject;
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
        textures: objectGroup.children.filter(m => m.material.map).length,
        annotationLoaded: annotationMesh !== null};
    },
  };
  let down = {x: 0, y: 0};
  const canvas = element.renderer.domElement;

  canvas.addEventListener('contextmenu', event => {
    if (workspaceMode === 'annotate') event.preventDefault();
  });

  window.addEventListener('keydown', event => {
    if (
      workspaceMode !== 'annotate'
      || !(event.ctrlKey || event.metaKey)
      || event.altKey
      || event.key.toLowerCase() !== 'z'
    ) return;

    const target = event.target;
    if (
      target instanceof HTMLInputElement
      || target instanceof HTMLTextAreaElement
      || target instanceof HTMLSelectElement
      || target?.isContentEditable
    ) return;

    event.preventDefault();
    if (annotationHistory.length) {
      annotationFaces = annotationHistory.pop();
      refreshAnnotation();
    }
  });

  canvas.addEventListener('pointerdown', event => {
    down = {x: event.clientX, y: event.clientY};
    beginAnnotationStroke(event);
  }, true);

  canvas.addEventListener('pointermove', event => {
    if (
      painting
      && event.pointerId === activePointerId
      && (event.buttons & 1)
    ) {
      paintAnnotation(event);
      event.preventDefault();
      event.stopPropagation();
    }
  }, true);

  canvas.addEventListener('pointerup', event => {
    if (endAnnotationStroke(event)) return;
    if (
      !data
      || workspaceMode === 'annotate'
      || Math.hypot(
        event.clientX - down.x,
        event.clientY - down.y,
      ) > 4
    ) return;

    if (displayMode !== 'all') return;
    const rect = canvas.getBoundingClientRect();
    const pointer = new THREE.Vector2(
      (event.clientX - rect.left) / rect.width * 2 - 1,
      -(event.clientY - rect.top) / rect.height * 2 + 1,
    );
    const ray = new THREE.Raycaster();
    ray.setFromCamera(pointer, element.camera);
    const hits = ray.intersectObjects(groups.map(group => group.all));
    if (hits.length && hits[0].instanceId !== undefined) {
      element.$emit('grasp_pick', hits[0].instanceId);
    }
  }, true);

  canvas.addEventListener('pointercancel', event => {
    endAnnotationStroke(event);
  }, true);
  return api;
}
