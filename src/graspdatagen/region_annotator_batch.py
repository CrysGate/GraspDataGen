from __future__ import annotations

import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import numpy as np


HTML = r"""<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>GraspDataGen Batch Region Annotator</title>

<style>
html, body {
  margin: 0;
  width: 100%;
  height: 100%;
  overflow: hidden;
  background: #202124;
  color: white;
  font-family: sans-serif;
}

#sidebar {
  position: absolute;
  left: 0;
  top: 0;
  bottom: 0;
  width: 220px;
  z-index: 20;
  background: rgba(18,18,18,0.96);
  border-right: 1px solid #444;
  overflow-y: auto;
}

#sidebarHeader {
  position: sticky;
  top: 0;
  background: #181818;
  padding: 12px;
  border-bottom: 1px solid #444;
  z-index: 2;
}

.asset {
  padding: 8px 10px;
  cursor: pointer;
  border-bottom: 1px solid #303030;
  font-size: 13px;
}

.asset:hover {
  background: #303236;
}

.asset.current {
  background: #3b4655;
}

.asset.done::before {
  content: "✓ ";
  color: #8ee59b;
}

.asset.todo::before {
  content: "○ ";
  color: #aaa;
}

#panel {
  position: absolute;
  top: 14px;
  left: 238px;
  z-index: 10;
  width: 300px;
  padding: 14px;
  color: white;
  background: rgba(20,20,20,0.90);
  border-radius: 10px;
  line-height: 1.55;
}

button {
  margin: 4px 3px 0 0;
  padding: 6px 10px;
}

input[type="range"] {
  width: 130px;
}

#name {
  color: #8fd3ff;
  font-weight: bold;
}

#status {
  margin-top: 8px;
  color: #9fe3a8;
  min-height: 22px;
}

#face {
  color: #ffcc66;
  font-weight: bold;
}

#dirty {
  color: #ffcc66;
}

canvas {
  display: block;
}
</style>

<script type="importmap">
{
  "imports": {
    "three": "https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.js",
    "three/addons/": "https://cdn.jsdelivr.net/npm/three@0.180.0/examples/jsm/"
  }
}
</script>
</head>

<body>

<div id="sidebar">
  <div id="sidebarHeader">
    <b>Assets</b><br>
    <span id="progress">Loading...</span>
  </div>
  <div id="assetList"></div>
</div>

<div id="panel">
  <b>Batch Region Annotator</b><br><br>

  Asset:
  <span id="name">-</span><br>

  Progress:
  <span id="position">-</span><br>

  State:
  <span id="annotationState">-</span>
  <span id="dirty"></span><br><br>

  左键拖动：涂选<br>
  Shift + 左键：擦除<br>
  右键拖动：旋转<br>
  滚轮：缩放<br>
  Ctrl + Z：撤销<br><br>

  当前 face:
  <span id="face">-</span><br>

  已选择:
  <span id="count">0</span> faces<br><br>

  Brush radius:
  <input id="brush"
         type="range"
         min="0.002"
         max="0.050"
         step="0.002"
         value="0.010">
  <span id="brushValue">10 mm</span><br><br>

  <button id="prev">← Previous</button>
  <button id="next">Next →</button><br>

  <button id="undo">Undo</button>
  <button id="clear">Clear</button><br>

  <button id="save">Save</button>
  <button id="saveNext">Save & Next →</button>

  <div id="status"></div>
</div>

<script type="module">

import * as THREE from "three";
import { OrbitControls } from
  "three/addons/controls/OrbitControls.js";


/* ---------- global state ---------- */

let assets = [];
let currentIndex = -1;

let vertices = [];
let faces = [];

let geometry = null;
let objectMesh = null;
let wire = null;
let overlay = null;

let neighbours = [];
let faceCenters = [];

const selectedFaces = new Set();

let dirty = false;

const undoStack = [];
const MAX_UNDO = 100;


/* ---------- scene ---------- */

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x202124);

const camera =
  new THREE.PerspectiveCamera(
    45,
    window.innerWidth / window.innerHeight,
    0.001,
    1000
  );

const renderer =
  new THREE.WebGLRenderer({
    antialias: true
  });

renderer.setPixelRatio(
  window.devicePixelRatio
);

renderer.setSize(
  window.innerWidth,
  window.innerHeight
);

document.body.appendChild(
  renderer.domElement
);

scene.add(
  new THREE.HemisphereLight(
    0xffffff,
    0x444444,
    2.2
  )
);

const keyLight =
  new THREE.DirectionalLight(
    0xffffff,
    2.0
  );

keyLight.position.set(1, 2, 3);
scene.add(keyLight);

const controls =
  new OrbitControls(
    camera,
    renderer.domElement
  );

controls.enableDamping = true;
controls.dampingFactor = 0.08;

controls.mouseButtons.LEFT = null;
controls.mouseButtons.RIGHT =
  THREE.MOUSE.ROTATE;
controls.mouseButtons.MIDDLE =
  THREE.MOUSE.DOLLY;

renderer.domElement.addEventListener(
  "contextmenu",
  e => e.preventDefault()
);


/* ---------- UI ---------- */

function setStatus(text) {
  document.getElementById(
    "status"
  ).textContent = text;
}

function setDirty(value) {

  dirty = value;

  document.getElementById(
    "dirty"
  ).textContent =
    dirty ? " (unsaved)" : "";
}

function updateProgress() {

  const done =
    assets.filter(a => a.annotated).length;

  document.getElementById(
    "progress"
  ).textContent =
    `${done} / ${assets.length} annotated`;

  document.getElementById(
    "position"
  ).textContent =
    currentIndex >= 0
      ? `${currentIndex + 1} / ${assets.length}`
      : "-";
}

function rebuildAssetList() {

  const list =
    document.getElementById(
      "assetList"
    );

  list.innerHTML = "";

  assets.forEach(
    (asset, index) => {

      const div =
        document.createElement("div");

      div.className =
        "asset "
        + (asset.annotated
            ? "done"
            : "todo")
        + (index === currentIndex
            ? " current"
            : "");

      div.textContent =
        asset.name;

      div.addEventListener(
        "click",
        () => loadAsset(index)
      );

      list.appendChild(div);
    }
  );

  updateProgress();
}


/* ---------- mesh disposal ---------- */

function disposeCurrentMesh() {

  if (overlay !== null) {

    scene.remove(overlay);

    overlay.geometry.dispose();
    overlay.material.dispose();

    overlay = null;
  }

  if (wire !== null) {

    scene.remove(wire);

    wire.geometry.dispose();
    wire.material.dispose();

    wire = null;
  }

  if (objectMesh !== null) {

    scene.remove(objectMesh);

    objectMesh.material.dispose();

    objectMesh = null;
  }

  if (geometry !== null) {

    geometry.dispose();
    geometry = null;
  }
}


/* ---------- adjacency ---------- */

function buildAdjacency() {

  neighbours =
    Array.from(
      { length: faces.length },
      () => new Set()
    );

  const edgeMap = new Map();

  function addEdge(a, b, faceId) {

    if (a > b) {
      const t = a;
      a = b;
      b = t;
    }

    const key = `${a},${b}`;

    if (!edgeMap.has(key)) {
      edgeMap.set(key, []);
    }

    edgeMap.get(key).push(faceId);
  }

  for (
    let i = 0;
    i < faces.length;
    i++
  ) {

    const [a, b, c] = faces[i];

    addEdge(a, b, i);
    addEdge(b, c, i);
    addEdge(c, a, i);
  }

  for (const ids of edgeMap.values()) {

    if (ids.length < 2) {
      continue;
    }

    for (const a of ids) {
      for (const b of ids) {

        if (a !== b) {
          neighbours[a].add(b);
        }
      }
    }
  }

  faceCenters =
    faces.map(f => {

      const a = vertices[f[0]];
      const b = vertices[f[1]];
      const c = vertices[f[2]];

      return [
        (a[0] + b[0] + c[0]) / 3,
        (a[1] + b[1] + c[1]) / 3,
        (a[2] + b[2] + c[2]) / 3
      ];
    });
}


/* ---------- selected overlay ---------- */

function rebuildOverlay() {

  if (overlay !== null) {

    scene.remove(overlay);

    overlay.geometry.dispose();
    overlay.material.dispose();

    overlay = null;
  }

  document.getElementById(
    "count"
  ).textContent =
    selectedFaces.size;

  if (selectedFaces.size === 0) {
    return;
  }

  const positions = [];

  for (const faceId of selectedFaces) {

    const f = faces[faceId];

    positions.push(
      ...vertices[f[0]],
      ...vertices[f[1]],
      ...vertices[f[2]]
    );
  }

  const selectedGeometry =
    new THREE.BufferGeometry();

  selectedGeometry.setAttribute(
    "position",
    new THREE.BufferAttribute(
      new Float32Array(positions),
      3
    )
  );

  const selectedMaterial =
    new THREE.MeshBasicMaterial({
      color: 0xff5533,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.82,
      polygonOffset: true,
      polygonOffsetFactor: -2,
      polygonOffsetUnits: -2
    });

  overlay =
    new THREE.Mesh(
      selectedGeometry,
      selectedMaterial
    );

  scene.add(overlay);
}


/* ---------- undo ---------- */

function snapshotSelection() {
  return new Set(selectedFaces);
}

function pushUndo(snapshot) {

  undoStack.push(snapshot);

  if (undoStack.length > MAX_UNDO) {
    undoStack.shift();
  }
}

function restoreSelection(snapshot) {

  selectedFaces.clear();

  for (const faceId of snapshot) {
    selectedFaces.add(faceId);
  }

  rebuildOverlay();
  setDirty(true);
}

function undo() {

  if (undoStack.length === 0) {
    setStatus("Nothing to undo.");
    return;
  }

  restoreSelection(
    undoStack.pop()
  );

  setStatus("Undone.");
}


/* ---------- load asset ---------- */

async function loadAsset(index) {

  if (
    index < 0
    || index >= assets.length
  ) {
    return;
  }

  if (
    dirty
    && !window.confirm(
      "Current annotation has unsaved changes. Discard them?"
    )
  ) {
    return;
  }

  setStatus("Loading...");

  const name =
    assets[index].name;

  const response =
    await fetch(
      `/mesh?name=${encodeURIComponent(name)}`
    );

  const data =
    await response.json();

  if (!response.ok) {

    setStatus(
      `Load failed: ${data.error}`
    );

    return;
  }

  disposeCurrentMesh();

  currentIndex = index;

  vertices = data.vertices;
  faces = data.faces;

  selectedFaces.clear();

  for (
    const faceId
    of data.allowed_faces
  ) {
    selectedFaces.add(faceId);
  }

  undoStack.length = 0;

  geometry =
    new THREE.BufferGeometry();

  geometry.setAttribute(
    "position",
    new THREE.BufferAttribute(
      new Float32Array(
        vertices.flat()
      ),
      3
    )
  );

  geometry.setIndex(
    new THREE.BufferAttribute(
      new Uint32Array(
        faces.flat()
      ),
      1
    )
  );

  geometry.computeVertexNormals();
  geometry.computeBoundingBox();

  objectMesh =
    new THREE.Mesh(
      geometry,
      new THREE.MeshStandardMaterial({
        color: 0xb8c4cc,
        roughness: 0.65,
        metalness: 0.0,
        side: THREE.DoubleSide
      })
    );

  scene.add(objectMesh);

  /*
    Very dense meshes make WireframeGeometry expensive.
    Keep it only for smaller prepared surfaces.
  */
  if (faces.length < 100000) {

    wire =
      new THREE.LineSegments(
        new THREE.WireframeGeometry(
          geometry
        ),
        new THREE.LineBasicMaterial({
          color: 0x444444,
          transparent: true,
          opacity: 0.20
        })
      );

    scene.add(wire);
  }

  buildAdjacency();
  rebuildOverlay();

  const box =
    geometry.boundingBox;

  const center =
    new THREE.Vector3();

  box.getCenter(center);

  const sizeVector =
    new THREE.Vector3();

  box.getSize(sizeVector);

  const size =
    Math.max(
      sizeVector.x,
      sizeVector.y,
      sizeVector.z
    );

  camera.position.set(
    center.x + size * 1.8,
    center.y + size * 1.3,
    center.z + size * 1.8
  );

  camera.near =
    Math.max(
      size / 1000,
      0.0001
    );

  camera.far =
    Math.max(
      size * 100,
      10
    );

  camera.updateProjectionMatrix();

  controls.target.copy(center);
  controls.update();

  document.getElementById(
    "name"
  ).textContent = name;

  document.getElementById(
    "annotationState"
  ).textContent =
    data.annotated
      ? "已标注"
      : "未标注";

  document.getElementById(
    "face"
  ).textContent = "-";

  setDirty(false);

  rebuildAssetList();

  setStatus(
    `${faces.length.toLocaleString()} faces loaded.`
  );
}


/* ---------- brush ---------- */

let brushRadius = 0.010;

const brush =
  document.getElementById(
    "brush"
  );

const brushValue =
  document.getElementById(
    "brushValue"
  );

brush.addEventListener(
  "input",
  () => {

    brushRadius =
      Number(brush.value);

    brushValue.textContent =
      `${Math.round(
        brushRadius * 1000
      )} mm`;
  }
);

function brushFaces(
  seed,
  clickPoint,
  radius
) {

  const result =
    new Set([seed]);

  const queue = [seed];

  while (queue.length > 0) {

    const current =
      queue.shift();

    for (
      const next
      of neighbours[current]
    ) {

      if (result.has(next)) {
        continue;
      }

      const center =
        faceCenters[next];

      const dx =
        center[0] - clickPoint[0];

      const dy =
        center[1] - clickPoint[1];

      const dz =
        center[2] - clickPoint[2];

      const distance =
        Math.sqrt(
          dx * dx
          + dy * dy
          + dz * dz
        );

      if (distance <= radius) {

        result.add(next);
        queue.push(next);
      }
    }
  }

  return result;
}


/* ---------- picking ---------- */

const raycaster =
  new THREE.Raycaster();

const mouse =
  new THREE.Vector2();

let painting = false;
let eraseMode = false;
let lastFace = -1;
let strokeSnapshot = null;

function getHit(event) {

  if (objectMesh === null) {
    return null;
  }

  const rect =
    renderer.domElement
      .getBoundingClientRect();

  mouse.x =
    ((event.clientX - rect.left)
      / rect.width) * 2 - 1;

  mouse.y =
    -((event.clientY - rect.top)
      / rect.height) * 2 + 1;

  raycaster.setFromCamera(
    mouse,
    camera
  );

  const hits =
    raycaster.intersectObject(
      objectMesh,
      false
    );

  return hits.length > 0
    ? hits[0]
    : null;
}

function paint(event) {

  const hit =
    getHit(event);

  if (hit === null) {
    return;
  }

  const faceId =
    hit.faceIndex;

  document.getElementById(
    "face"
  ).textContent =
    faceId;

  if (faceId === lastFace) {
    return;
  }

  lastFace = faceId;

  const affected =
    brushFaces(
      faceId,
      [
        hit.point.x,
        hit.point.y,
        hit.point.z
      ],
      brushRadius
    );

  if (eraseMode) {

    for (const f of affected) {
      selectedFaces.delete(f);
    }

  } else {

    for (const f of affected) {
      selectedFaces.add(f);
    }
  }

  rebuildOverlay();
  setDirty(true);
}

renderer.domElement.addEventListener(
  "pointerdown",
  event => {

    if (event.button !== 0) {
      return;
    }

    painting = true;

    eraseMode =
      event.shiftKey;

    lastFace = -1;

    strokeSnapshot =
      snapshotSelection();

    renderer.domElement
      .setPointerCapture(
        event.pointerId
      );

    paint(event);
  }
);

renderer.domElement.addEventListener(
  "pointermove",
  event => {

    if (!painting) {
      return;
    }

    eraseMode =
      event.shiftKey;

    paint(event);
  }
);

renderer.domElement.addEventListener(
  "pointerup",
  event => {

    if (event.button !== 0) {
      return;
    }

    painting = false;
    lastFace = -1;

    if (strokeSnapshot !== null) {

      let changed =
        strokeSnapshot.size
        !== selectedFaces.size;

      if (!changed) {

        for (
          const faceId
          of strokeSnapshot
        ) {

          if (
            !selectedFaces.has(faceId)
          ) {
            changed = true;
            break;
          }
        }
      }

      if (changed) {
        pushUndo(strokeSnapshot);
      }

      strokeSnapshot = null;
    }

    try {

      renderer.domElement
        .releasePointerCapture(
          event.pointerId
        );

    } catch (_) {}
  }
);


/* ---------- save ---------- */

async function saveCurrent(
  goNext = false
) {

  if (currentIndex < 0) {
    return;
  }

  if (selectedFaces.size === 0) {

    setStatus(
      "Nothing selected."
    );

    return;
  }

  setStatus("Saving...");

  const response =
    await fetch(
      "/save",
      {
        method: "POST",
        headers: {
          "Content-Type":
            "application/json"
        },
        body: JSON.stringify({
          name:
            assets[currentIndex].name,

          allowed_faces:
            Array.from(
              selectedFaces
            ).sort(
              (a, b) => a - b
            )
        })
      }
    );

  const result =
    await response.json();

  if (!response.ok) {

    setStatus(
      `Save failed: ${result.error}`
    );

    return;
  }

  assets[currentIndex]
    .annotated = true;

  document.getElementById(
    "annotationState"
  ).textContent =
    "已标注";

  setDirty(false);

  rebuildAssetList();

  setStatus(
    `Saved ${result.count} faces.`
  );

  if (
    goNext
    && currentIndex + 1
       < assets.length
  ) {

    await loadAsset(
      currentIndex + 1
    );
  }
}


/* ---------- buttons ---------- */

document.getElementById(
  "prev"
).addEventListener(
  "click",
  () => loadAsset(
    currentIndex - 1
  )
);

document.getElementById(
  "next"
).addEventListener(
  "click",
  () => loadAsset(
    currentIndex + 1
  )
);

document.getElementById(
  "undo"
).addEventListener(
  "click",
  undo
);

document.getElementById(
  "clear"
).addEventListener(
  "click",
  () => {

    if (
      selectedFaces.size === 0
    ) {
      return;
    }

    pushUndo(
      snapshotSelection()
    );

    selectedFaces.clear();

    rebuildOverlay();
    setDirty(true);

    setStatus(
      "Selection cleared."
    );
  }
);

document.getElementById(
  "save"
).addEventListener(
  "click",
  () => saveCurrent(false)
);

document.getElementById(
  "saveNext"
).addEventListener(
  "click",
  () => saveCurrent(true)
);


/* ---------- keyboard ---------- */

window.addEventListener(
  "keydown",
  event => {

    if (
      (event.ctrlKey
       || event.metaKey)
      && event.key
        .toLowerCase() === "z"
      && !event.shiftKey
    ) {

      event.preventDefault();
      undo();
    }
  }
);

window.addEventListener(
  "beforeunload",
  event => {

    if (!dirty) {
      return;
    }

    event.preventDefault();
    event.returnValue = "";
  }
);


/* ---------- resize ---------- */

window.addEventListener(
  "resize",
  () => {

    camera.aspect =
      window.innerWidth
      / window.innerHeight;

    camera.updateProjectionMatrix();

    renderer.setSize(
      window.innerWidth,
      window.innerHeight
    );
  }
);


/* ---------- render ---------- */

function animate() {

  requestAnimationFrame(
    animate
  );

  controls.update();

  renderer.render(
    scene,
    camera
  );
}

animate();


/* ---------- startup ---------- */

const assetsResponse =
  await fetch("/assets");

const assetsData =
  await assetsResponse.json();

if (!assetsResponse.ok) {

  setStatus(
    `Asset scan failed: ${assetsData.error}`
  );

} else {

  assets = assetsData.assets;

  rebuildAssetList();

  if (assets.length > 0) {

    let first =
      assets.findIndex(
        a => !a.annotated
      );

    if (first < 0) {
      first = 0;
    }

    await loadAsset(first);
  }
}

</script>
</body>
</html>
"""


def load_geometry(
    path: Path,
) -> tuple[np.ndarray, np.ndarray]:

    with np.load(
        path,
        allow_pickle=False,
    ) as data:

        vertices = np.asarray(
            data["surface_vertices_m"],
            dtype=np.float64,
        )

        faces = np.asarray(
            data["surface_faces"],
            dtype=np.int64,
        )

    return vertices, faces


def scan_assets(
    prepared_root: Path,
) -> dict[str, Path]:

    grouped: dict[
        str,
        list[tuple[Path, str | None]],
    ] = {}

    for manifest_path in sorted(
        prepared_root.glob(
            "*/manifest.json"
        )
    ):

        geometry_path = (
            manifest_path.parent
            / "geometry.npz"
        )

        if not geometry_path.is_file():
            continue

        manifest = json.loads(
            manifest_path.read_text()
        )

        name = manifest["name"]

        geometry_hash = (
            manifest
            .get("artifacts", {})
            .get("geometry.npz")
        )

        grouped.setdefault(
            name,
            [],
        ).append(
            (
                geometry_path,
                geometry_hash,
            )
        )

    assets: dict[str, Path] = {}

    for name, entries in grouped.items():

        hashes = {
            geometry_hash
            for _, geometry_hash
            in entries
        }

        if len(hashes) > 1:

            raise RuntimeError(
                f"{name}: duplicate prepared "
                "caches have different "
                "geometry hashes"
            )

        assets[name] = entries[0][0]

    return dict(
        sorted(
            assets.items()
        )
    )


def serve(
    prepared_root: Path,
    output_dir: Path,
    host: str,
    port: int,
) -> None:

    assets = scan_assets(
        prepared_root
    )

    if not assets:
        raise RuntimeError(
            "No prepared objects found"
        )

    class Handler(
        BaseHTTPRequestHandler
    ):

        def send_json(
            self,
            payload: dict,
            status: int = 200,
        ) -> None:

            body = json.dumps(
                payload,
                separators=(",", ":"),
            ).encode("utf-8")

            self.send_response(status)

            self.send_header(
                "Content-Type",
                "application/json",
            )

            self.send_header(
                "Content-Length",
                str(len(body)),
            )

            self.end_headers()

            self.wfile.write(body)

        def send_html(
            self,
        ) -> None:

            body = HTML.encode("utf-8")

            self.send_response(200)

            self.send_header(
                "Content-Type",
                "text/html; charset=utf-8",
            )

            self.send_header(
                "Content-Length",
                str(len(body)),
            )

            self.end_headers()

            self.wfile.write(body)

        def do_GET(self) -> None:

            parsed = urlparse(
                self.path
            )

            if parsed.path == "/":

                self.send_html()
                return

            if parsed.path == "/assets":

                self.send_json(
                    {
                        "assets": [
                            {
                                "name": name,
                                "annotated":
                                    (
                                        output_dir
                                        / f"{name}.npz"
                                    ).is_file(),
                            }
                            for name
                            in assets
                        ]
                    }
                )

                return

            if parsed.path == "/mesh":

                try:

                    query = parse_qs(
                        parsed.query
                    )

                    name = query.get(
                        "name",
                        [None],
                    )[0]

                    if (
                        name is None
                        or name not in assets
                    ):

                        raise ValueError(
                            "unknown asset"
                        )

                    geometry_path = (
                        assets[name]
                    )

                    vertices, faces = \
                        load_geometry(
                            geometry_path
                        )

                    annotation_path = (
                        output_dir
                        / f"{name}.npz"
                    )

                    allowed_faces = \
                        np.empty(
                            0,
                            dtype=np.int64,
                        )

                    annotated = False

                    if annotation_path.is_file():

                        with np.load(
                            annotation_path,
                            allow_pickle=False,
                        ) as data:

                            ann_vertices = \
                                np.asarray(
                                    data[
                                        "surface_vertices_m"
                                    ]
                                )

                            ann_faces = \
                                np.asarray(
                                    data[
                                        "surface_faces"
                                    ]
                                )

                            if (
                                not np.array_equal(
                                    ann_vertices,
                                    vertices,
                                )
                                or
                                not np.array_equal(
                                    ann_faces,
                                    faces,
                                )
                            ):

                                raise ValueError(
                                    "existing annotation "
                                    "does not match "
                                    "prepared topology"
                                )

                            allowed_faces = \
                                np.asarray(
                                    data[
                                        "allowed_faces"
                                    ],
                                    dtype=np.int64,
                                )

                        annotated = True

                    self.send_json(
                        {
                            "name": name,
                            "annotated":
                                annotated,
                            "vertices":
                                vertices.tolist(),
                            "faces":
                                faces.tolist(),
                            "allowed_faces":
                                allowed_faces.tolist(),
                        }
                    )

                except Exception as exc:

                    self.send_json(
                        {
                            "error":
                                str(exc)
                        },
                        status=400,
                    )

                return

            self.send_error(404)

        def do_POST(self) -> None:

            if self.path != "/save":

                self.send_error(404)
                return

            try:

                length = int(
                    self.headers.get(
                        "Content-Length",
                        "0",
                    )
                )

                payload = json.loads(
                    self.rfile
                    .read(length)
                    .decode("utf-8")
                )

                name = payload["name"]

                if name not in assets:

                    raise ValueError(
                        "unknown asset"
                    )

                vertices, faces = \
                    load_geometry(
                        assets[name]
                    )

                allowed_faces = np.asarray(
                    payload[
                        "allowed_faces"
                    ],
                    dtype=np.int64,
                )

                if allowed_faces.ndim != 1:

                    raise ValueError(
                        "allowed_faces "
                        "must be 1-D"
                    )

                if len(allowed_faces) == 0:

                    raise ValueError(
                        "allowed_faces is empty"
                    )

                allowed_faces = np.unique(
                    allowed_faces
                )

                if (
                    allowed_faces.min() < 0
                    or
                    allowed_faces.max()
                    >= len(faces)
                ):

                    raise ValueError(
                        "face id out of range"
                    )

                output_dir.mkdir(
                    parents=True,
                    exist_ok=True,
                )

                output_path = (
                    output_dir
                    / f"{name}.npz"
                )

                np.savez_compressed(
                    output_path,
                    allowed_faces=
                        allowed_faces,
                    surface_vertices_m=
                        vertices,
                    surface_faces=
                        faces,
                )

                print(
                    f"Saved {name}: "
                    f"{len(allowed_faces)} "
                    f"faces -> {output_path}",
                    flush=True,
                )

                self.send_json(
                    {
                        "ok": True,
                        "count":
                            int(
                                len(
                                    allowed_faces
                                )
                            ),
                        "path":
                            str(
                                output_path
                            ),
                    }
                )

            except Exception as exc:

                self.send_json(
                    {
                        "ok": False,
                        "error": str(exc),
                    },
                    status=400,
                )

        def log_message(
            self,
            format: str,
            *args,
        ) -> None:

            print(
                "[web] "
                f"{self.address_string()} "
                f"{format % args}",
                flush=True,
            )

    server = ThreadingHTTPServer(
        (host, port),
        Handler,
    )

    annotated_count = sum(
        (
            output_dir
            / f"{name}.npz"
        ).is_file()
        for name in assets
    )

    print(
        "Batch region annotator: "
        f"http://{host}:{port}",
        flush=True,
    )

    print(
        f"prepared root: {prepared_root}",
        flush=True,
    )

    print(
        f"output dir:    {output_dir}",
        flush=True,
    )

    print(
        f"unique assets: {len(assets)}",
        flush=True,
    )

    print(
        f"annotated:     "
        f"{annotated_count}/{len(assets)}",
        flush=True,
    )

    try:

        server.serve_forever()

    except KeyboardInterrupt:

        print(
            "\nStopping batch annotator."
        )

    finally:

        server.server_close()


def main() -> None:

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--prepared-root",
        type=Path,
        default=Path(
            "outputs/prepared/objects"
        ),
    )

    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(
            "annotations/grasp_regions"
        ),
    )

    parser.add_argument(
        "--host",
        default="127.0.0.1",
    )

    parser.add_argument(
        "--port",
        type=int,
        default=8765,
    )

    args = parser.parse_args()

    serve(
        args.prepared_root
        .expanduser()
        .resolve(),

        args.output_dir
        .expanduser()
        .resolve(),

        args.host,
        args.port,
    )


if __name__ == "__main__":
    main()
