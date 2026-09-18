from __future__ import annotations

import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np


HTML = r"""<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>GraspDataGen Region Annotator</title>

<style>
html, body {
  margin: 0;
  width: 100%;
  height: 100%;
  overflow: hidden;
  background: #202124;
  font-family: sans-serif;
}

#panel {
  position: absolute;
  top: 14px;
  left: 14px;
  z-index: 10;
  width: 260px;
  padding: 14px;
  color: white;
  background: rgba(20,20,20,0.88);
  border-radius: 10px;
  line-height: 1.55;
}

button {
  margin: 4px 3px 0 0;
  padding: 6px 12px;
}

input[type="range"] {
  width: 130px;
}

#status {
  margin-top: 7px;
  color: #9fe3a8;
}

#face {
  color: #ffcc66;
  font-weight: bold;
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

<div id="panel">
  <b>Grasp Region Annotator</b><br><br>

  左键拖动：涂选<br>
  Shift + 左键：擦除<br>
  右键拖动：旋转<br>
  滚轮：缩放<br><br>

  当前 face:
  <span id="face">-</span><br>

  已选择:
  <span id="count">0</span> faces<br><br>

  Brush radius:
  <input id="brush" type="range"
         min="0.002" max="0.050" step="0.002" value="0.010">
  <span id="brushValue">10 mm</span><br>

  <button id="undo">Undo</button>
  <button id="clear">Clear</button>
  <button id="save">Save</button>

  <div id="status"></div>
</div>

<script type="module">

import * as THREE from "three";
import { OrbitControls } from
  "three/addons/controls/OrbitControls.js";


/* ---------- load mesh ---------- */

const response = await fetch("/mesh");
const meshData = await response.json();

const vertices = meshData.vertices;
const faces = meshData.faces;


/* ---------- scene ---------- */

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x202124);

const camera = new THREE.PerspectiveCamera(
  45,
  window.innerWidth / window.innerHeight,
  0.001,
  1000
);

const renderer = new THREE.WebGLRenderer({
  antialias: true
});

renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(
  window.innerWidth,
  window.innerHeight
);

document.body.appendChild(renderer.domElement);


/* ---------- geometry ---------- */

const geometry = new THREE.BufferGeometry();

geometry.setAttribute(
  "position",
  new THREE.BufferAttribute(
    new Float32Array(vertices.flat()),
    3
  )
);

geometry.setIndex(
  new THREE.BufferAttribute(
    new Uint32Array(faces.flat()),
    1
  )
);

geometry.computeVertexNormals();
geometry.computeBoundingBox();

const material = new THREE.MeshStandardMaterial({
  color: 0xb8c4cc,
  roughness: 0.65,
  metalness: 0.0,
  side: THREE.DoubleSide
});

const objectMesh = new THREE.Mesh(
  geometry,
  material
);

scene.add(objectMesh);


/* ---------- wireframe ---------- */

const wire = new THREE.LineSegments(
  new THREE.WireframeGeometry(geometry),
  new THREE.LineBasicMaterial({
    color: 0x444444,
    transparent: true,
    opacity: 0.20
  })
);

scene.add(wire);


/* ---------- light ---------- */

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


/* ---------- camera ---------- */

const box = geometry.boundingBox;

const center = new THREE.Vector3();
box.getCenter(center);

const sizeVector = new THREE.Vector3();
box.getSize(sizeVector);

const size = Math.max(
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
  Math.max(size / 1000, 0.0001);

camera.far =
  Math.max(size * 100, 10);

camera.updateProjectionMatrix();


/* ---------- controls ---------- */

const controls =
  new OrbitControls(
    camera,
    renderer.domElement
  );

controls.target.copy(center);

controls.enableDamping = true;
controls.dampingFactor = 0.08;

/*
  Left mouse is reserved for painting.
  Right mouse rotates.
*/
controls.mouseButtons.LEFT = null;
controls.mouseButtons.RIGHT =
  THREE.MOUSE.ROTATE;
controls.mouseButtons.MIDDLE =
  THREE.MOUSE.DOLLY;

controls.update();

renderer.domElement.addEventListener(
  "contextmenu",
  e => e.preventDefault()
);


/* ---------- face adjacency ---------- */

const neighbours =
  Array.from(
    { length: faces.length },
    () => new Set()
  );

const edgeMap = new Map();

function addEdge(a, b, faceId) {

  if (a > b) {
    const temp = a;
    a = b;
    b = temp;
  }

  const key = `${a},${b}`;

  if (!edgeMap.has(key)) {
    edgeMap.set(key, []);
  }

  edgeMap.get(key).push(faceId);
}

for (let i = 0; i < faces.length; i++) {

  const [a, b, c] = faces[i];

  addEdge(a, b, i);
  addEdge(b, c, i);
  addEdge(c, a, i);
}

for (const ids of edgeMap.values()) {

  if (ids.length < 2) continue;

  for (const a of ids) {
    for (const b of ids) {

      if (a !== b) {
        neighbours[a].add(b);
      }
    }
  }
}



/* ---------- face centers ---------- */

const faceCenters = faces.map(f => {

  const a = vertices[f[0]];
  const b = vertices[f[1]];
  const c = vertices[f[2]];

  return [
    (a[0] + b[0] + c[0]) / 3,
    (a[1] + b[1] + c[1]) / 3,
    (a[2] + b[2] + c[2]) / 3
  ];
});


/* ---------- selected faces ---------- */

const selectedFaces = new Set();

/*
  Undo history stores one snapshot per user action.

  One complete left-drag paint/erase stroke counts as one action,
  rather than every individual face touched during the drag.
*/
const undoStack = [];
const MAX_UNDO = 100;

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
}

function undo() {

  if (undoStack.length === 0) {

    document.getElementById(
      "status"
    ).textContent =
      "Nothing to undo.";

    return;
  }

  restoreSelection(
    undoStack.pop()
  );

  document.getElementById(
    "status"
  ).textContent =
    "Undone.";
}

let overlay = null;

function rebuildOverlay() {

  if (overlay !== null) {

    scene.remove(overlay);

    overlay.geometry.dispose();
    overlay.material.dispose();

    overlay = null;
  }

  if (selectedFaces.size === 0) {

    document.getElementById(
      "count"
    ).textContent = "0";

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

  overlay = new THREE.Mesh(
    selectedGeometry,
    selectedMaterial
  );

  scene.add(overlay);

  document.getElementById(
    "count"
  ).textContent =
    selectedFaces.size;
}


/* ---------- brush ---------- */

let brushRadius = 0.010;

const brush =
  document.getElementById("brush");

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


/*
  Select only faces that are:

  1. connected to the clicked face
  2. inside the spatial brush radius

  This prevents the brush from travelling arbitrarily far
  through long / skinny triangles.
*/
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
      const next of neighbours[current]
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
          dx * dx +
          dy * dy +
          dz * dz
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

  if (hits.length === 0) {
    return null;
  }

  return hits[0];
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

  const clickPoint = [
    hit.point.x,
    hit.point.y,
    hit.point.z
  ];

  const affected =
    brushFaces(
      faceId,
      clickPoint,
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

    // Save the state before this entire brush stroke.
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

        for (const faceId of strokeSnapshot) {

          if (!selectedFaces.has(faceId)) {
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


/* ---------- undo ---------- */

document.getElementById(
  "undo"
).addEventListener(
  "click",
  undo
);


/* ---------- clear ---------- */

document.getElementById(
  "clear"
).addEventListener(
  "click",
  () => {

    if (selectedFaces.size === 0) {
      return;
    }

    pushUndo(
      snapshotSelection()
    );

    selectedFaces.clear();

    rebuildOverlay();

    document.getElementById(
      "status"
    ).textContent =
      "Selection cleared.";
  }
);


/* ---------- save ---------- */

document.getElementById(
  "save"
).addEventListener(
  "click",
  async () => {

    const status =
      document.getElementById(
        "status"
      );

    if (
      selectedFaces.size === 0
    ) {

      status.textContent =
        "Nothing selected.";

      return;
    }

    status.textContent =
      "Saving...";

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

    if (response.ok) {

      status.textContent =
        `Saved ${result.count} faces`;

    } else {

      status.textContent =
        `Save failed: ${result.error}`;
    }
  }
);


/* ---------- keyboard shortcuts ---------- */

window.addEventListener(
  "keydown",
  event => {

    if (
      (event.ctrlKey || event.metaKey)
      && event.key.toLowerCase() === "z"
      && !event.shiftKey
    ) {

      event.preventDefault();
      undo();
    }
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

</script>
</body>
</html>
"""


def load_geometry(path: Path) -> tuple[np.ndarray, np.ndarray]:

    if not path.is_file():
        raise FileNotFoundError(path)

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


def serve(
    geometry_path: Path,
    output_path: Path,
    host: str,
    port: int,
) -> None:

    vertices, faces = \
        load_geometry(geometry_path)

    mesh_json = json.dumps(
        {
            "vertices":
                vertices.tolist(),

            "faces":
                faces.tolist(),
        },
        separators=(",", ":"),
    ).encode("utf-8")

    class Handler(
        BaseHTTPRequestHandler
    ):

        def send_bytes(
            self,
            body: bytes,
            content_type: str,
        ) -> None:

            self.send_response(200)

            self.send_header(
                "Content-Type",
                content_type,
            )

            self.send_header(
                "Content-Length",
                str(len(body)),
            )

            self.end_headers()

            self.wfile.write(body)


        def do_GET(self) -> None:

            if self.path == "/":

                self.send_bytes(
                    HTML.encode("utf-8"),
                    "text/html; charset=utf-8",
                )

                return

            if self.path == "/mesh":

                self.send_bytes(
                    mesh_json,
                    "application/json",
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

                raw = self.rfile.read(length)

                payload = json.loads(
                    raw.decode("utf-8")
                )

                allowed_faces = np.asarray(
                    payload["allowed_faces"],
                    dtype=np.int64,
                )

                if allowed_faces.ndim != 1:

                    raise ValueError(
                        "allowed_faces must be 1-D"
                    )

                if len(allowed_faces) == 0:

                    raise ValueError(
                        "allowed_faces is empty"
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

                allowed_faces = np.unique(
                    allowed_faces
                )

                output_path.parent.mkdir(
                    parents=True,
                    exist_ok=True,
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
                    f"Saved {len(allowed_faces)} "
                    f"faces -> {output_path}",
                    flush=True,
                )

                body = json.dumps(
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
                ).encode("utf-8")

                self.send_bytes(
                    body,
                    "application/json",
                )

            except Exception as exc:

                body = json.dumps(
                    {
                        "ok": False,
                        "error": str(exc),
                    }
                ).encode("utf-8")

                self.send_response(400)

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


        def log_message(
            self,
            format: str,
            *args,
        ) -> None:

            print(
                f"[web] "
                f"{self.address_string()} "
                f"{format % args}",
                flush=True,
            )


    server = ThreadingHTTPServer(
        (host, port),
        Handler,
    )

    print(
        f"Region annotator: "
        f"http://{host}:{port}",
        flush=True,
    )

    print(
        f"geometry: {geometry_path}",
        flush=True,
    )

    print(
        f"output:   {output_path}",
        flush=True,
    )

    print(
        f"mesh: {len(vertices)} vertices, "
        f"{len(faces)} faces",
        flush=True,
    )

    try:

        server.serve_forever()

    except KeyboardInterrupt:

        print(
            "\nStopping region annotator."
        )

    finally:

        server.server_close()


def main() -> None:

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "geometry",
        type=Path,
        help="Prepared geometry.npz",
    )

    parser.add_argument(
        "--name",
        required=True,
        help="Object name, e.g. bottle",
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

    geometry_path = (
        args.geometry
        .expanduser()
        .resolve()
    )

    output_path = (
        args.output_dir
        / f"{args.name}.npz"
    ).expanduser().resolve()

    serve(
        geometry_path,
        output_path,
        args.host,
        args.port,
    )


if __name__ == "__main__":
    main()
