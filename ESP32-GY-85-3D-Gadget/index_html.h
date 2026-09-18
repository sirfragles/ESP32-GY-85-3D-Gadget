// Auto-generated from index.html — do not edit by hand.
// Regenerate with: python3 tools/gen_index_html.py
// Source: index.html, 6119 bytes, sha256 041178489a7f…
#pragma once

const char index_html[] PROGMEM = R"HTML(
<html>
  <head>
    <title>GY-85 3D web visualisation</title>
    <style>
      html,
      body {
        margin: 0;
        height: 100%;
      }
      #view {
        width: 100%;
        height: 100%;
      }
      #gauges {
        position: absolute;
        top: 8px;
        left: 8px;
        display: flex;
        gap: 8px;
      }
      #controls {
        position: absolute;
        top: 8px;
        right: 8px;
        display: flex;
        gap: 8px;
        align-items: center;
        color: #eee;
        font-family: sans-serif;
        font-size: 14px;
      }
    </style>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/101/three.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/canvas-gauges@2.1.7/gauge.min.js"></script>
    <script>
      var scene;
      var camera;
      var renderer;

      var cube;

      var gaugeRoll;
      var gaugePitch;
      var gaugeYaw;

      var ws = null;
      var calStatus = null;

      function render() {
        requestAnimationFrame(render);
        renderer.render(scene, camera);
      }

      function CubeBegin() {
        scene = new THREE.Scene();
        camera = new THREE.PerspectiveCamera(
          75,
          window.innerWidth / window.innerHeight,
          0.1,
          1000
        );
        renderer = new THREE.WebGLRenderer();

        renderer.setSize(window.innerWidth, window.innerHeight);
        renderer.domElement.id = "view";
        document.body.appendChild(renderer.domElement);
        var geometry = new THREE.BoxGeometry(100, 100, 100);
        var cubeMaterials = [
          new THREE.MeshBasicMaterial({ color: 0xfe4365 }),
          new THREE.MeshBasicMaterial({ color: 0xfc9d9a }),
          new THREE.MeshBasicMaterial({ color: 0xf9cdad }),
          new THREE.MeshBasicMaterial({ color: 0xc8cba9 }),
          new THREE.MeshBasicMaterial({ color: 0x83af98 }),
          new THREE.MeshBasicMaterial({ color: 0xe5fcc2 })
        ];

        cube = new THREE.Mesh(geometry, cubeMaterials);
        scene.add(cube);

        camera.position.z = 200;

        render();
      }

      function GaugeBegin() {
        gaugeRoll = new RadialGauge({
          renderTo: "gauge-roll",
          width: 140,
          height: 140,
          minValue: -180,
          maxValue: 180,
          majorTicks: ["-180", "-90", "0", "90", "180"],
          minorTicks: 5,
          title: "Roll"
        }).draw();
        gaugePitch = new RadialGauge({
          renderTo: "gauge-pitch",
          width: 140,
          height: 140,
          minValue: -90,
          maxValue: 90,
          majorTicks: ["-90", "-45", "0", "45", "90"],
          minorTicks: 5,
          title: "Pitch"
        }).draw();
        gaugeYaw = new RadialGauge({
          renderTo: "gauge-yaw",
          width: 140,
          height: 140,
          minValue: -180,
          maxValue: 180,
          majorTicks: ["-180", "-90", "0", "90", "180"],
          minorTicks: 5,
          title: "Yaw"
        }).draw();
      }
      function WebSocketBegin() {
        CubeBegin();
        GaugeBegin();

        calStatus = document.getElementById("cal-status");

        if ("WebSocket" in window) {
          connect();
        } else {
          // The browser doesn't support WebSocket
          alert("WebSocket NOT supported by your Browser!");
        }
      }

      // The board sleeps when idle and comes back after a wake-up, so keep
      // reconnecting instead of alerting when the connection drops.
      function connect() {
        try {
          // The page and the websocket are served directly by the ESP32.
          ws = new WebSocket("ws://" + location.hostname + ":8001/");
        } catch (e) {
          ws = null;
          // no host name (page opened from disk) - retry in a moment
          setTimeout(connect, 2000);
          return;
        }

        ws.onmessage = function(evt) {
          //create a JSON object
          var jsonObject = JSON.parse(evt.data);

          // calibration status messages from the firmware
          if (jsonObject.cal !== undefined) {
            if (jsonObject.cal === "started") {
              calStatus.textContent = "calibrating... keep it still";
            } else if (jsonObject.cal === "done") {
              calStatus.textContent = "calibrated";
            } else if (jsonObject.cal === "aborted") {
              calStatus.textContent = "moved - try again";
            } else {
              calStatus.textContent = "place flat and still, then retry";
            }
            return;
          }

          if (jsonObject.q0 === undefined) return;  // not a sensor frame

          var q0 = jsonObject.q0;
          var q1 = jsonObject.q1;
          var q2 = jsonObject.q2;
          var q3 = jsonObject.q3;

          var quat1 = new THREE.Quaternion(q1, q2, q3, q0);
          var quat2 = new THREE.Quaternion(1, 0, 0, 0);

          cube.quaternion.multiplyQuaternions(quat1, quat2);

          if (jsonObject.roll !== undefined) {
            // undefined for firmware that only sends quaternions
            gaugeRoll.value = jsonObject.roll;
            gaugePitch.value = jsonObject.pitch;
            gaugeYaw.value = jsonObject.yaw;
          }
        };

        ws.onclose = function() {
          setTimeout(connect, 2000);
        };

        ws.onerror = function() {
          ws.close();
        };
      }

      // "Calibrate" button: ask the board for a manual zero-offset refresh.
      function calibrate() {
        if (ws && ws.readyState === WebSocket.OPEN) {
          calStatus.textContent = "requesting...";
          ws.send("calibrate");
        } else {
          calStatus.textContent = "not connected";
        }
      }
    </script>
  </head>

  <body onLoad="javascript:WebSocketBegin()">
    <div id="gauges">
      <canvas id="gauge-roll"></canvas>
      <canvas id="gauge-pitch"></canvas>
      <canvas id="gauge-yaw"></canvas>
    </div>
    <div id="controls">
      <button onclick="calibrate()">Calibrate</button>
      <span id="cal-status"></span>
    </div>
  </body>
</html>
)HTML";
