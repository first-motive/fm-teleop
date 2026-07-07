"use strict";
// In-browser 3D arm viewer — three.js + urdf-loader, driven live over the same Foxglove
// WebSocket bridge as the rest of the GUI. Replaces the separate Foxglove Studio window.
//
// Globals expected (loaded as classic scripts before this file, file://-safe):
//   THREE, THREE.STLLoader, THREE.ColladaLoader, THREE.OrbitControls, URDFLoader
//
// API (window.Viewer3D):
//   init(canvas)                       one-time scene/renderer/camera setup
//   setURDF(urdfXml, fetchAsset)       parse URDF; fetchAsset(uri)->Promise<Uint8Array> loads meshes
//   setJoints(names, positions)        drive joint angles (FK from the URDF)
//   setTarget(pos, quat)               place the /target_pose gizmo ([x,y,z], [x,y,z,w])
//   setActive(on)                      start/stop the render loop (only render when the pane is shown)
//   resize()                           match the canvas to its container
//   ready()                            true once the URDF + meshes are loaded

window.Viewer3D = (function () {
  let renderer, scene, camera, controls, canvas;
  let robot = null, target = null, active = false, rafId = 0, loaded = false;

  // Camera framing mirrors foxglove/arm_3d.json (Z-up, ~3/4 view around the arm base).
  const TARGET = new THREE.Vector3(0, 0, 0.15);

  function init(cvs) {
    if (renderer) return;
    canvas = cvs;
    // ROS is Z-up; make three.js agree so OrbitControls + the URDF read upright.
    THREE.Object3D.DEFAULT_UP = new THREE.Vector3(0, 0, 1);

    renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));

    scene = new THREE.Scene();

    camera = new THREE.PerspectiveCamera(50, 1, 0.01, 100);
    camera.up.set(0, 0, 1);
    camera.position.set(1.1, -1.1, 1.0);

    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    controls.target.copy(TARGET);

    scene.add(new THREE.HemisphereLight(0xffffff, 0x2a2530, 0.9));
    const dir = new THREE.DirectionalLight(0xffffff, 0.9);
    dir.position.set(1, -1, 2);
    scene.add(dir);

    // ground grid on the XY plane (GridHelper is XZ by default -> rotate into Z-up)
    const grid = new THREE.GridHelper(2, 20, 0x4a4550, 0x35313b);
    grid.rotation.x = Math.PI / 2;
    scene.add(grid);

    window.addEventListener("resize", () => { if (active) resize(); });
  }

  function clearRobot() {
    if (robot) { scene.remove(robot); robot = null; }
    loaded = false;
  }

  function setURDF(urdfXml, fetchAsset) {
    if (!renderer) return;
    clearRobot();
    const manager = new THREE.LoadingManager();
    manager.onLoad = () => { loaded = true; frameRobot(); };

    const loader = new URDFLoader(manager);
    loader.parseVisual = true;
    loader.parseCollision = false;
    // Keep the original package:// URI so loadMeshCb can fetch it over the WS bridge.
    loader.packages = (pkg) => "package://" + pkg;
    loader.loadMeshCb = (path, mgr, done) => {
      fetchAsset(path).then((bytes) => {
        const buf = bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength);
        let obj = null;
        if (/\.stl$/i.test(path)) {
          const geom = new THREE.STLLoader().parse(buf);
          geom.computeVertexNormals();
          obj = new THREE.Mesh(geom, new THREE.MeshStandardMaterial(
            { color: 0xd8d2c4, metalness: 0.25, roughness: 0.6 }));
        } else if (/\.dae$/i.test(path)) {
          const dae = new THREE.ColladaLoader(mgr).parse(new TextDecoder().decode(bytes), path);
          obj = dae.scene;
        }
        done(obj);
      }).catch((e) => { console.warn("[3d] mesh load failed", path, e); done(null); });
    };

    try {
      robot = loader.parse(urdfXml);
      scene.add(robot);
    } catch (e) {
      console.error("[3d] URDF parse failed", e);
    }
  }

  // Orient/frame the loaded robot: urdf-loader keeps ROS Z-up, which matches our scene.
  function frameRobot() {
    if (!robot) return;
    controls.target.copy(TARGET);
    controls.update();
  }

  function setJoints(names, positions) {
    if (!robot || !robot.setJointValue) return;
    for (let i = 0; i < names.length; i++) {
      if (robot.joints && robot.joints[names[i]]) robot.setJointValue(names[i], positions[i]);
    }
  }

  function setTarget(pos, quat) {
    if (!renderer) return;
    if (!target) { target = new THREE.AxesHelper(0.12); scene.add(target); }
    target.position.set(pos[0], pos[1], pos[2]);
    target.quaternion.set(quat[0], quat[1], quat[2], quat[3]);
  }

  function resize() {
    if (!renderer || !canvas) return;
    const w = canvas.clientWidth || canvas.parentElement.clientWidth;
    const h = canvas.clientHeight || canvas.parentElement.clientHeight;
    if (!w || !h) return;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }

  function loop() {
    if (!active) return;
    rafId = requestAnimationFrame(loop);
    controls.update();
    renderer.render(scene, camera);
  }

  function setActive(on) {
    active = on;
    if (on) { resize(); if (!rafId) loop(); }
    else if (rafId) { cancelAnimationFrame(rafId); rafId = 0; }
  }

  return { init, setURDF, setJoints, setTarget, setActive, resize, ready: () => loaded };
})();
