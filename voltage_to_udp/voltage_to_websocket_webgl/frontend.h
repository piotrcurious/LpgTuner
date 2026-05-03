#ifndef FRONTEND_H
#define FRONTEND_H

const char* html_index = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <title>PRO AUTO SCOPE</title>
  <style>
    body { background: #000; color: #0f0; font-family: 'Consolas', 'Courier New', monospace; margin: 0; overflow: hidden; }
    #ui { position: absolute; top: 10px; left: 10px; z-index: 10; background: rgba(20,20,20,0.8); padding: 15px; border: 1px solid #444; border-radius: 4px; box-shadow: 0 0 10px rgba(0,255,0,0.2); pointer-events: auto; max-height: 90vh; overflow-y: auto; width: 220px; }
    canvas { width: 100vw; height: 100vh; display: block; position: absolute; top: 0; left: 0; }
    .chan { margin-bottom: 8px; font-size: 14px; }
    .chan-row { display: flex; align-items: center; margin-bottom: 5px; }
    .chan-color { width: 12px; height: 12px; margin-right: 8px; border-radius: 2px; }
    .c0 { background: #ffff00; } .c1 { background: #00ffff; } .c2 { background: #ff00ff; } .c3 { background: #00ff00; }
    .chan-metrics { font-size: 9px; color: #888; margin-top: 2px; display: grid; grid-template-columns: 1fr 1fr; }
    .btn { background: #333; color: #eee; border: 1px solid #666; padding: 4px 8px; cursor: pointer; margin-right: 5px; font-family: inherit; font-size: 11px; }
    .btn:hover { background: #555; }
    .btn.active { background: #006600; border-color: #00ff00; }
    .btn-stop.active { background: #660000; border-color: #ff0000; }
    .stat-overlay { position: absolute; bottom: 10px; right: 15px; color: #008800; font-size: 11px; text-shadow: 1px 1px #000; pointer-events: none; text-align: right; line-height: 1.4; }
    .cursor-stats { position: absolute; top: 10px; right: 15px; color: #fff; font-size: 12px; background: rgba(0,0,0,0.5); padding: 5px; border: 1px solid #444; pointer-events: none; }
    .trigger-indicator { display: inline-block; width: 8px; height: 8px; border-radius: 50%; background: #333; margin-left: 5px; }
    .trigger-active { background: #f00; box-shadow: 0 0 5px #f00; }
    h2 { margin: 0 0 10px 0; font-size: 14px; color: #aaa; border-bottom: 1px solid #333; padding-bottom: 5px; }
    .ctrl-group { margin-top: 10px; border-top: 1px solid #333; padding-top: 5px; }
    label { font-size: 10px; color: #888; display: block; margin-bottom: 2px; }
    input[type=range] { width: 100%; height: 4px; margin-bottom: 8px; accent-color: #0f0; }
    .val-display { float: right; color: #0f0; }
    .status-bit { font-size: 10px; color: #f0f; font-weight: bold; margin-left: 10px; }
    .trig-status { font-size: 11px; color: #fff; background: #440; padding: 2px 5px; border-radius: 2px; margin-left: 10px; }
    .trig-active { background: #080; }
    .trig-level-label { position: absolute; right: 0; color: #f00; font-size: 10px; background: rgba(0,0,0,0.6); padding: 1px 3px; pointer-events: none; border: 1px solid #600; display: none; z-index: 5; }
    .trig-pos-marker { position: absolute; top: 0; width: 1px; height: 100%; background: rgba(255,0,0,0.3); pointer-events: none; z-index: 2; }
    .trig-pos-marker::after { content: 'T'; color: #f00; font-size: 10px; position: absolute; top: 2px; left: 2px; }
    .loss-critical { color: #f00; text-shadow: 0 0 5px #f00; }
    #reconnect-btn { background: #600; border-color: #f00; margin-top: 10px; display: none; }
  </style>
</head>
<body>
  <canvas id="canvas"></canvas>
  <div id="trig-pos-marker" class="trig-pos-marker"></div>
  <div id="trig-level-label" class="trig-level-label">T: 1.65V</div>
  <div id="ui">
    <h2>ENGINE ANALYZER <span id="trig-state" class="trig-status">WAITING</span><span id="mode-bit" class="status-bit"></span></h2>
    <div id="chan-controls"></div>

    <div class="ctrl-group">
      <label>TIMEBASE (Horizontal Zoom) <span class="val-display" id="val-tb">1.0</span></label>
      <input type="range" id="ctrl-tb" min="0.1" max="5.0" step="0.1" value="1.0" oninput="updateControls()">
      <label>SAMPLE RATE (Hz) <span class="val-display" id="val-sr">50000</span></label>
      <select id="ctrl-sr" class="btn" onchange="updateSampleRate()" style="width:100%; font-size:10px; height:22px;">
        <option value="10000">10 kHz</option>
        <option value="20000">20 kHz</option>
        <option value="50000" selected>50 kHz</option>
        <option value="100000">100 kHz</option>
      </select>
    </div>

    <div class="ctrl-group">
      <div style="margin-bottom: 5px; font-size: 10px; color: #888;">RUN CONTROL</div>
      <button id="btnRun" class="btn active" onclick="toggleRun()">STOP</button>
      <select id="runMode" class="btn" onchange="updateRunMode()" style="font-size:10px; height:24px;">
        <option value="0">AUTO</option>
        <option value="1">NORMAL</option>
        <option value="2">SINGLE</option>
      </select>
    </div>

    <div class="ctrl-group">
      <div style="margin-bottom: 5px; font-size: 10px; color: #888;">TRIGGER CONFIG</div>
      <div style="display:flex; gap:2px; margin-bottom:5px;">
        <button id="btnType0" class="btn active" onclick="setTriggerType(0)">IND</button>
        <button id="btnType1" class="btn" onclick="setTriggerType(1)">CAM</button>
        <button id="btnEdgeR" class="btn active" onclick="setEdge(1)">RISE</button>
        <button id="btnEdgeF" class="btn" onclick="setEdge(0)">FALL</button>
      </div>
      <div style="margin-bottom:5px;">
        <label>ANALOG SRC</label>
        <select id="analogCh" class="btn" onchange="updateAnalogTrig()" style="width:100%; font-size:10px; height:22px; background:#222; border:1px solid #444; color:#0f0;">
          <option value="-1">DISABLED</option>
          <option value="0">CH1</option>
          <option value="1">CH2</option>
          <option value="2">CH3</option>
          <option value="3">CH4</option>
        </select>
        <label>LEVEL <span class="val-display" id="val-atrig">1.65V</span></label>
        <input type="range" id="ctrl-atrig" min="0" max="4095" value="2048" oninput="updateAnalogTrig()">
      </div>
      <div style="margin-bottom:5px;">
         <label>HORIZ POS <span class="val-display" id="val-hpos">10%</span></label>
         <input type="range" id="ctrl-hpos" min="0" max="100" value="10" oninput="updateHPos()">
      </div>
      <div style="margin-top:5px;">
        <button id="btnTriggerNow" class="btn" onclick="forceTrigger()">FORCE</button>
        <button id="btnAutoScale" class="btn" onclick="autoScale()">AUTO</button>
      </div>
      <div style="margin-top:5px; font-size:9px; color:#888;">MASK:
        <input type="checkbox" id="m0" checked onclick="updateMask()">1
        <input type="checkbox" id="m1" checked onclick="updateMask()">2
        <input type="checkbox" id="m2" checked onclick="updateMask()">3
        <input type="checkbox" id="m3" checked onclick="updateMask()">4
      </div>
    </div>

    <div class="ctrl-group">
      <div style="margin-bottom: 5px; font-size: 10px; color: #888;">VIEW / CURSORS</div>
      <div style="display:flex; gap:2px; margin-bottom:5px;">
        <button id="btnCursors" class="btn" onclick="toggleCursors()">CURSORS</button>
        <button class="btn" onclick="exportCSV()">CSV</button>
      </div>
      <label>PERSISTENCE <span class="val-display" id="val-persist">8</span></label>
      <input type="range" id="ctrl-persist" min="1" max="16" step="1" value="8" oninput="updatePersistence()">
    </div>

    <div class="ctrl-group">
      <div style="margin-bottom: 5px; font-size: 10px; color: #888;">MATH CHANNEL</div>
      <select id="mathOp" class="btn" onchange="render()" style="width:100%; font-size:10px; height:22px; background:#222; border:1px solid #444; color:#f0f;">
        <option value="none">OFF</option>
        <option value="add">CH1 + CH2</option>
        <option value="sub">CH1 - CH2</option>
        <option value="mul">CH1 * CH2</option>
      </select>
    </div>

    <div class="ctrl-group">
      <div style="margin-bottom: 5px; font-size: 10px; color: #888;">DEBUG / SIMULATION</div>
      <button id="btnSim" class="btn" onclick="toggleSim()">SIMULATION: OFF</button>
      <label>NOISE LEVEL <span class="val-display" id="val-noise">0</span></label>
      <input type="range" id="ctrl-noise" min="0" max="100" step="1" value="0" oninput="updateNoise()">
    </div>

    <div class="ctrl-group">
      <div style="margin-bottom: 5px; font-size: 10px; color: #888;">WIFI CONFIG</div>
      <input style="background:#111;color:#0f0;border:1px solid #444;width:80px;font-size:10px;" type="text" id="ssid" placeholder="SSID">
      <input style="background:#111;color:#0f0;border:1px solid #444;width:80px;font-size:10px;" type="password" id="pass" placeholder="PASS">
      <button class="btn" onclick="saveWifi()">REBOOT</button>
    </div>

    <button id="reconnect-btn" class="btn" onclick="connect()">FORCE RECONNECT</button>
  </div>
  <div id="cursor-stats" class="cursor-stats" style="display:none"></div>
  <div class="stat-overlay" id="stats">
    LINK: DOWN | FPS: 0 | SEQ: 0<br>
    HEAP: 0 | LOSS: 0 | UP: 0s<br>
    LOAD: 0% | JITTER: 0us
  </div>

  <script id="vs" type="x-shader/x-vertex">
    attribute float a_y;
    attribute float a_x;
    uniform float u_offset;
    uniform float u_scale;
    uniform float u_tb;
    void main() {
      // a_y is now in millivolts (0-3300)
      gl_Position = vec4(a_x * u_tb, (a_y / 3300.0 * 2.0 - 1.0) * u_scale + u_offset, 0.0, 1.0);
    }
  </script>
  <script id="fs" type="x-shader/x-fragment">
    precision mediump float;
    uniform vec4 u_color;
    void main() {
      gl_FragColor = u_color;
    }
  </script>

  <script>
    const ADC_SAMPLE_FREQ_HZ = 50000;
    const canvas = document.getElementById('canvas');
    let gl;
    try {
      gl = canvas.getContext('webgl', { antialias: true });
    } catch(e) {
      alert("WebGL not supported");
    }

    let ws, simOn = false, lastSeq = -1, lossCount = 0, startTime = Date.now();
    let reconnectTimeout;

    function createShader(gl, type, source) {
      const s = gl.createShader(type);
      gl.shaderSource(s, source);
      gl.compileShader(s);
      return s;
    }

    const program = gl.createProgram();
    gl.attachShader(program, createShader(gl, gl.VERTEX_SHADER, document.getElementById('vs').text));
    gl.attachShader(program, createShader(gl, gl.FRAGMENT_SHADER, document.getElementById('fs').text));
    gl.linkProgram(program);
    gl.useProgram(program);

    const SAMPLES = 512;
    const CHANNELS = 4;
    let phosphor_count = 8;
    let triggerPos = 10; // %
    let analogTrigCh = -1;
    let analogTrigLevel = 2048;

    let metricsEMA = Array.from({length: CHANNELS}, () => ({ pp: 0, rms: 0 }));
    const EMA_ALPHA = 0.2;

    let history = [];
    const xCoords = new Float32Array(SAMPLES);
    for(let i=0; i<SAMPLES; i++) xCoords[i] = (i / (SAMPLES-1)) * 2.0 - 1.0;

    const xBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, xBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, xCoords, gl.STATIC_DRAW);

    const colors = [ [1,1,0,1], [0,1,1,1], [1,0,1,1], [0,1,0,1] ];
    const mathColor = [1,0,1,1];
    let chanSettings = [
      { offset: 0.6, scale: 0.25 },
      { offset: 0.2, scale: 0.25 },
      { offset: -0.2, scale: 0.25 },
      { offset: -0.6, scale: 0.25 }
    ];

    function initUI() {
      const container = document.getElementById('chan-controls');
      container.innerHTML = '';
      for(let i=0; i<CHANNELS; i++) {
        const div = document.createElement('div');
        div.className = 'chan';
        div.innerHTML = `
          <div class="chan-row">
            <div class="chan-color c${i}"></div>
            <span>CH${i+1}: <span id="v${i}">0.00</span>V</span>
            <div id="t${i}" class="trigger-indicator"></div>
          </div>
          <div class="chan-metrics">
            <span>P-P: <span id="pp${i}">0.0</span>V</span>
            <span>RMS: <span id="rms${i}">0.0</span>V</span>
          </div>
          <label>SCALE <span class="val-display" id="val-s${i}">0.25</span></label>
          <input type="range" id="ctrl-s${i}" min="0.05" max="1.0" step="0.05" value="0.25" oninput="updateControls()">
          <label>OFFSET <span class="val-display" id="val-o${i}">${chanSettings[i].offset}</span></label>
          <input type="range" id="ctrl-o${i}" min="-1.0" max="1.0" step="0.05" value="${chanSettings[i].offset}" oninput="updateControls()">
        `;
        container.appendChild(div);
      }
    }

    function updateControls() {
      for(let i=0; i<CHANNELS; i++) {
        chanSettings[i].scale = parseFloat(document.getElementById('ctrl-s'+i).value);
        chanSettings[i].offset = parseFloat(document.getElementById('ctrl-o'+i).value);
        document.getElementById('val-s'+i).innerText = chanSettings[i].scale.toFixed(2);
        document.getElementById('val-o'+i).innerText = chanSettings[i].offset.toFixed(2);
      }
      document.getElementById('val-tb').innerText = document.getElementById('ctrl-tb').value;
    }

    function updateNoise() {
      const n = document.getElementById('ctrl-noise').value;
      document.getElementById('val-noise').innerText = n;
      if(ws && ws.readyState === WebSocket.OPEN) ws.send('N' + n);
    }

    function updatePersistence() {
      phosphor_count = parseInt(document.getElementById('ctrl-persist').value);
      document.getElementById('val-persist').innerText = phosphor_count;
      while(history.length > phosphor_count) history.shift();
    }

    const gridBuffer = gl.createBuffer();
    function createGrid() {
      const gridPoints = [];
      for(let i=-10; i<=10; i++) {
        gridPoints.push(-1, (i/10 + 1)*1650, 1, (i/10 + 1)*1650);
        gridPoints.push(i/10, 0, i/10, 3300);
      }
      gl.bindBuffer(gl.ARRAY_BUFFER, gridBuffer);
      gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(gridPoints), gl.STATIC_DRAW);
    }

    function drawTriggerLine() {
      const label = document.getElementById('trig-level-label');
      if (analogTrigCh < 0) {
        label.style.display = 'none';
        return;
      }
      gl.uniform1f(gl.getUniformLocation(program, 'u_offset'), chanSettings[analogTrigCh].offset);
      gl.uniform1f(gl.getUniformLocation(program, 'u_scale'), chanSettings[analogTrigCh].scale);
      gl.uniform1f(gl.getUniformLocation(program, 'u_tb'), 1);
      gl.uniform4f(gl.getUniformLocation(program, 'u_color'), 1, 0, 0, 0.4);

      const mv = (analogTrigLevel / 4095.0) * 3300.0;
      const pts = new Float32Array([-1, mv, 1, mv]);
      const b = gl.createBuffer();
      gl.bindBuffer(gl.ARRAY_BUFFER, b);
      gl.bufferData(gl.ARRAY_BUFFER, pts, gl.STREAM_DRAW);
      gl.vertexAttribPointer(gl.getAttribLocation(program, 'a_x'), 1, gl.FLOAT, false, 8, 0);
      gl.vertexAttribPointer(gl.getAttribLocation(program, 'a_y'), 1, gl.FLOAT, false, 8, 4);
      gl.drawArrays(gl.LINES, 0, 2);
      gl.deleteBuffer(b);

      // Update Label
      const normY = ((mv / 3300.0 * 2.0 - 1.0) * chanSettings[analogTrigCh].scale + chanSettings[analogTrigCh].offset);
      const canvasY = (1.0 - (normY + 1.0) / 2.0) * canvas.height;
      label.style.display = 'block';
      label.style.top = canvasY + 'px';
      label.innerText = 'T: ' + (analogTrigLevel * 3.3 / 4095).toFixed(2) + 'V';
    }

    function drawGrid() {
      gl.bindBuffer(gl.ARRAY_BUFFER, gridBuffer);
      const a_x = gl.getAttribLocation(program, 'a_x');
      const a_y = gl.getAttribLocation(program, 'a_y');
      gl.enableVertexAttribArray(a_x);
      gl.vertexAttribPointer(a_x, 1, gl.FLOAT, false, 8, 0);
      gl.enableVertexAttribArray(a_y);
      gl.vertexAttribPointer(a_y, 1, gl.FLOAT, false, 8, 4);

      gl.uniform4f(gl.getUniformLocation(program, 'u_color'), 0.1, 0.1, 0.1, 1);
      gl.uniform1f(gl.getUniformLocation(program, 'u_offset'), 0);
      gl.uniform1f(gl.getUniformLocation(program, 'u_scale'), 1);
      gl.uniform1f(gl.getUniformLocation(program, 'u_tb'), 1);
      gl.drawArrays(gl.LINES, 0, 44);
    }

    let cursorsOn = false;
    let cursorX = [0.2, 0.8], cursorY = [0.3, 0.7];
    let activeCursor = null;

    function toggleCursors() {
      cursorsOn = !cursorsOn;
      document.getElementById('btnCursors').innerText = 'CURSORS: ' + (cursorsOn ? 'ON' : 'OFF');
      document.getElementById('btnCursors').classList.toggle('active', cursorsOn);
      document.getElementById('cursor-stats').style.display = cursorsOn ? 'block' : 'none';
      render();
    }

    function updateCursorStats() {
      if (!cursorsOn) return;
      const dx = Math.abs(cursorX[1] - cursorX[0]);
      const dy = Math.abs(cursorY[1] - cursorY[0]);
      const dt = dx * (SAMPLES / ADC_SAMPLE_FREQ_HZ * 1000); // approx ms based on view
      const dv = dy * 3.3;
      document.getElementById('cursor-stats').innerHTML = `
        ΔX: ${dt.toFixed(3)}ms | ${(1000/dt).toFixed(1)}Hz<br>
        ΔY: ${dv.toFixed(3)}V
      `;
    }

    function drawCursors() {
      if (!cursorsOn) return;
      updateCursorStats();
      gl.uniform1f(gl.getUniformLocation(program, 'u_offset'), 0);
      gl.uniform1f(gl.getUniformLocation(program, 'u_scale'), 1);
      gl.uniform1f(gl.getUniformLocation(program, 'u_tb'), 1);
      gl.uniform4f(gl.getUniformLocation(program, 'u_color'), 1, 1, 1, 0.3);

      const a_x = gl.getAttribLocation(program, 'a_x');
      const a_y = gl.getAttribLocation(program, 'a_y');

      // Vertical cursors
      cursorX.forEach(x => {
        const pts = new Float32Array([x*2-1, 0, x*2-1, 4095]);
        const b = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, b);
        gl.bufferData(gl.ARRAY_BUFFER, pts, gl.STREAM_DRAW);
        gl.vertexAttribPointer(a_x, 1, gl.FLOAT, false, 8, 0);
        gl.vertexAttribPointer(a_y, 1, gl.FLOAT, false, 8, 4);
        gl.drawArrays(gl.LINES, 0, 2);
      });

      // Horizontal cursors
      cursorY.forEach(y => {
        const pts = new Float32Array([-1, y*3300, 1, y*3300]);
        const b = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, b);
        gl.bufferData(gl.ARRAY_BUFFER, pts, gl.STREAM_DRAW);
        gl.vertexAttribPointer(a_x, 1, gl.FLOAT, false, 8, 0);
        gl.vertexAttribPointer(a_y, 1, gl.FLOAT, false, 8, 4);
        gl.drawArrays(gl.LINES, 0, 2);
      });
    }

    function render() {
      gl.viewport(0, 0, canvas.width, canvas.height);
      gl.clearColor(0.01, 0.01, 0.01, 1);
      gl.clear(gl.COLOR_BUFFER_BIT);
      gl.enable(gl.BLEND);
      gl.blendFunc(gl.SRC_ALPHA, gl.ONE);

      drawGrid();
      drawTriggerLine();

      if (history.length === 0) {
        drawCursors();
        return;
      }

      const a_x = gl.getAttribLocation(program, 'a_x');
      const a_y = gl.getAttribLocation(program, 'a_y');
      const u_color = gl.getUniformLocation(program, 'u_color');
      const u_offset = gl.getUniformLocation(program, 'u_offset');
      const u_scale = gl.getUniformLocation(program, 'u_scale');
      const u_tb = gl.getUniformLocation(program, 'u_tb');

      const tbVal = parseFloat(document.getElementById('ctrl-tb').value);

      for (let h = 0; h < history.length; h++) {
        const entry = history[h];
        const alpha = (h + 1) / history.length;

        if (!entry.bufs) {
          entry.bufs = Array.from({length: CHANNELS}, () => gl.createBuffer());
          for (let c = 0; c < CHANNELS; c++) {
            const chanData = new Float32Array(SAMPLES);
            for(let i=0; i<SAMPLES; i++) chanData[i] = entry.data[i * CHANNELS + c];
            gl.bindBuffer(gl.ARRAY_BUFFER, entry.bufs[c]);
            gl.bufferData(gl.ARRAY_BUFFER, chanData, gl.STATIC_DRAW);
          }
        }

        for (let c = 0; c < CHANNELS; c++) {
          gl.enableVertexAttribArray(a_x);
          gl.bindBuffer(gl.ARRAY_BUFFER, xBuffer);
          gl.vertexAttribPointer(a_x, 1, gl.FLOAT, false, 0, 0);
          gl.enableVertexAttribArray(a_y);
          gl.bindBuffer(gl.ARRAY_BUFFER, entry.bufs[c]);
          gl.vertexAttribPointer(a_y, 1, gl.FLOAT, false, 0, 0);

          const color = [...colors[c]];
          color[3] = alpha * 0.6;
          gl.uniform4fv(u_color, color);
          gl.uniform1f(u_offset, chanSettings[c].offset);
          gl.uniform1f(u_scale, chanSettings[c].scale);
          gl.uniform1f(u_tb, tbVal);
          gl.drawArrays(gl.LINE_STRIP, 0, SAMPLES);

        }

        const mathOp = document.getElementById('mathOp').value;
        if (mathOp !== 'none') {
           const mathData = new Float32Array(SAMPLES);
           for(let i=0; i<SAMPLES; i++) {
              const v1 = entry.data[i * CHANNELS + 0]; // mV
              const v2 = entry.data[i * CHANNELS + 1]; // mV
              if (mathOp === 'add') mathData[i] = (v1 + v2) / 2;
              else if (mathOp === 'sub') mathData[i] = (v1 - v2) + 1650;
              else if (mathOp === 'mul') mathData[i] = (v1 * v2) / 3300.0;
           }
           const mathBuf = gl.createBuffer();
           gl.bindBuffer(gl.ARRAY_BUFFER, mathBuf);
           gl.bufferData(gl.ARRAY_BUFFER, mathData, gl.STREAM_DRAW);
           gl.vertexAttribPointer(a_x, 1, gl.FLOAT, false, 0, 0);
           gl.bindBuffer(gl.ARRAY_BUFFER, mathBuf);
           gl.vertexAttribPointer(a_y, 1, gl.FLOAT, false, 0, 0);

           const mCol = [...mathColor];
           mCol[3] = alpha * 0.8;
           gl.uniform4fv(u_color, mCol);
           gl.uniform1f(u_offset, 0);
           gl.uniform1f(u_scale, 0.5);
           gl.drawArrays(gl.LINE_STRIP, 0, SAMPLES);
           gl.deleteBuffer(mathBuf);
        }

        for (let c = 0; c < CHANNELS; c++) {
          if (h === history.length - 1) {
            let min = 3300, max = 0, sum = 0, sumSq = 0;
            for(let i=0; i<SAMPLES; i++) {
              const val = entry.data[i * CHANNELS + c];
              if(val < min) min = val;
              if(val > max) max = val;
              const v = val / 1000.0;
              sumSq += v*v;
            }

            const pp = (max - min) / 1000.0;
            const rms = Math.sqrt(sumSq / SAMPLES);

            metricsEMA[c].pp = metricsEMA[c].pp * (1 - EMA_ALPHA) + pp * EMA_ALPHA;
            metricsEMA[c].rms = metricsEMA[c].rms * (1 - EMA_ALPHA) + rms * EMA_ALPHA;

            document.getElementById('v'+c).innerText = (entry.data[0 * CHANNELS + c] / 1000.0).toFixed(2);
            document.getElementById('pp'+c).innerText = metricsEMA[c].pp.toFixed(2);
            document.getElementById('rms'+c).innerText = metricsEMA[c].rms.toFixed(2);

            const tInd = document.getElementById('t'+c);
            if (entry.trig & (1 << c)) {
              tInd.classList.add('trigger-active');
              setTimeout(() => tInd.classList.remove('trigger-active'), 100);
            }
          }
        }
      }
      drawCursors();
    }

    canvas.onmousedown = (e) => {
      if (!cursorsOn) return;
      const x = e.clientX / canvas.width;
      const y = 1.0 - (e.clientY / canvas.height);

      for(let i=0; i<2; i++) {
        if(Math.abs(x - cursorX[i]) < 0.02) { activeCursor = {type:'x', idx:i}; return; }
        if(Math.abs(y - cursorY[i]) < 0.02) { activeCursor = {type:'y', idx:i}; return; }
      }
    };

    window.onmousemove = (e) => {
      if (!activeCursor) return;
      if (activeCursor.type === 'x') cursorX[activeCursor.idx] = e.clientX / canvas.width;
      else cursorY[activeCursor.idx] = 1.0 - (e.clientY / canvas.height);
      render();
    };

    window.onmouseup = () => activeCursor = null;

    let pktCount = 0, lastTime = performance.now(), fps = 0;

    function connect() {
      if(ws) ws.close();
      const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
      ws = new WebSocket(protocol + '//' + location.host + '/ws');
      ws.binaryType = 'arraybuffer';

      ws.onopen = () => {
        document.getElementById('stats').innerText = 'LINK: UP';
        document.getElementById('reconnect-btn').style.display = 'none';
        if(reconnectTimeout) clearTimeout(reconnectTimeout);
      };

      ws.onclose = () => {
        document.getElementById('stats').innerText = 'LINK: DOWN';
        document.getElementById('reconnect-btn').style.display = 'block';
        reconnectTimeout = setTimeout(connect, 5000);
      };

      ws.onmessage = (e) => {
        if (typeof e.data === 'string') return;
        const view = new DataView(e.data);
        const seq = view.getUint32(0, true);
        const trig = view.getUint16(8, true);
        const trigIdx = view.getUint16(10, true);
        const heap = view.getUint32(12, true);
        const loopTime = view.getUint16(16, true);
        const interval = view.getUint16(18, true);
        const data = new Uint16Array(e.data, 20, SAMPLES * CHANNELS);

        if(lastSeq !== -1 && seq !== lastSeq + 1) lossCount += (seq - lastSeq - 1);
        lastSeq = seq;

        history.push({seq, trig, data});
        if(history.length > phosphor_count) {
          const old = history.shift();
          if(old.bufs) old.bufs.forEach(b => gl.deleteBuffer(b));
        }

        pktCount++;
        const now = performance.now();
        if(now - lastTime > 1000) {
          fps = pktCount; pktCount = 0; lastTime = now;
          const lossClass = (lossCount > 100) ? "loss-critical" : "";
          const load = Math.round((loopTime / interval) * 100);
          const expected_interval = (SAMPLES / ADC_SAMPLE_FREQ_HZ) * 1000000;
          document.getElementById('stats').innerHTML = `
            LINK: UP | FPS: ${fps} | SEQ: ${seq}<br>
            HEAP: ${heap} | <span class="${lossClass}">LOSS: ${lossCount}</span> | UP: ${Math.floor((Date.now()-startTime)/1000)}s<br>
            LOAD: ${load}% | JITTER: ${Math.abs(interval - expected_interval).toFixed(0)}us
          `;
          document.getElementById('mode-bit').innerText = (trig & (1 << 7)) ? "[CAM MODE]" : "";
        }

        const trigStateEl = document.getElementById('trig-state');
        if (trig & 0x1F) {
           trigStateEl.innerText = "TRIG'D";
           trigStateEl.classList.add('trig-active');
           setTimeout(() => { trigStateEl.innerText = "WAITING"; trigStateEl.classList.remove('trig-active'); }, 500);
        }
        requestAnimationFrame(render);
      };
    }

    let isRunning = true;
    function toggleRun() {
      isRunning = !isRunning;
      document.getElementById('btnRun').innerText = isRunning ? 'STOP' : 'RUN';
      document.getElementById('btnRun').classList.toggle('btn-stop', isRunning);
      document.getElementById('btnRun').classList.toggle('active', isRunning);
      if(ws && ws.readyState === WebSocket.OPEN) ws.send('F' + (isRunning ? '1' : '0'));
    }

    function updateRunMode() {
      const m = document.getElementById('runMode').value;
      if(ws && ws.readyState === WebSocket.OPEN) ws.send('R' + m);
    }

    function setTriggerType(m) {
      if(ws && ws.readyState === WebSocket.OPEN) {
        ws.send('M' + m);
        document.getElementById('btnType0').classList.toggle('active', m === 0);
        document.getElementById('btnType1').classList.toggle('active', m === 1);
      }
    }

    function setEdge(e) {
      if(ws && ws.readyState === WebSocket.OPEN) {
        ws.send('E' + e);
        document.getElementById('btnEdgeR').classList.toggle('active', e === 1);
        document.getElementById('btnEdgeF').classList.toggle('active', e === 0);
      }
    }

    function updateAnalogTrig() {
       analogTrigCh = parseInt(document.getElementById('analogCh').value);
       analogTrigLevel = parseInt(document.getElementById('ctrl-atrig').value);
       document.getElementById('val-atrig').innerText = (analogTrigLevel * 3.3 / 4095).toFixed(2) + 'V';
       if(ws && ws.readyState === WebSocket.OPEN) {
          const buf = new Uint8Array(4);
          buf[0] = 'A'.charCodeAt(0);
          buf[1] = analogTrigCh;
          buf[2] = (analogTrigLevel >> 8) & 0xFF;
          buf[3] = analogTrigLevel & 0xFF;
          ws.send(buf);
       }
    }

    function updateSampleRate() {
      const hz = document.getElementById('ctrl-sr').value;
      document.getElementById('val-sr').innerText = hz;
      if(ws && ws.readyState === WebSocket.OPEN) ws.send('H' + hz);
    }

    function updateHPos() {
       triggerPos = parseInt(document.getElementById('ctrl-hpos').value);
       document.getElementById('val-hpos').innerText = triggerPos + '%';
       document.getElementById('trig-pos-marker').style.left = triggerPos + '%';
       if(ws && ws.readyState === WebSocket.OPEN) {
          const buf = new Uint8Array(2);
          buf[0] = 'P'.charCodeAt(0);
          buf[1] = triggerPos;
          ws.send(buf);
       }
    }

    function updateMask() {
      let mask = 0;
      for(let i=0; i<4; i++) if(document.getElementById('m'+i).checked) mask |= (1 << i);
      if(ws && ws.readyState === WebSocket.OPEN) {
        const buf = new Uint8Array(2);
        buf[0] = 'K'.charCodeAt(0);
        buf[1] = mask;
        ws.send(buf);
      }
    }

    function forceTrigger() {
      if(ws && ws.readyState === WebSocket.OPEN) ws.send('T');
    }

    function autoScale() {
      if (history.length === 0) return;
      const latest = history[history.length - 1].data;
      for (let c = 0; c < CHANNELS; c++) {
        let min = 3300, max = 0;
        for (let i = 0; i < SAMPLES; i++) {
          const val = latest[i * CHANNELS + c];
          if (val < min) min = val;
          if (val > max) max = val;
        }
        const range = max - min;
        if (range > 10) {
          const mid = (max + min) / 2;
          chanSettings[c].scale = 1.8 / (range / 3300); // Aim for ~90% height
          if (chanSettings[c].scale > 5.0) chanSettings[c].scale = 5.0; // Limit
          chanSettings[c].offset = -(mid / 3300 * 2 - 1) * chanSettings[c].scale;

          document.getElementById('ctrl-s' + c).value = chanSettings[c].scale;
          document.getElementById('ctrl-o' + c).value = chanSettings[c].offset;
        }
      }
      updateControls();
      render();
    }

    function toggleSim() {
      simOn = !simOn;
      if(ws && ws.readyState === WebSocket.OPEN) {
        ws.send('S' + (simOn ? '1' : '0'));
        document.getElementById('btnSim').innerText = 'SIMULATION: ' + (simOn ? 'ON' : 'OFF');
        document.getElementById('btnSim').classList.toggle('active', simOn);
      }
    }

    function saveWifi() {
      const s = document.getElementById('ssid').value, p = document.getElementById('pass').value;
      if(ws && ws.readyState === WebSocket.OPEN) ws.send('W;' + s + ';' + p);
    }

    function exportCSV() {
      if (history.length === 0) return;
      const latest = history[history.length - 1];
      let csv = "Sample,CH1,CH2,CH3,CH4\n";
      for (let i = 0; i < SAMPLES; i++) {
        csv += i;
        for (let c = 0; c < CHANNELS; c++) {
          csv += "," + (latest.data[i * CHANNELS + c] / 1000.0).toFixed(4);
        }
        csv += "\n";
      }
      const blob = new Blob([csv], { type: 'text/csv' });
      const url = window.URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.setAttribute('hidden', '');
      a.setAttribute('href', url);
      a.setAttribute('download', 'scope_capture.csv');
      document.body.appendChild(a);
      a.click();
      document.body.removeChild(a);
    }

    window.onresize = () => { canvas.width = window.innerWidth; canvas.height = window.innerHeight; gl.viewport(0, 0, canvas.width, canvas.height); };
    window.onresize();
    initUI();
    createGrid();
    updateHPos();
    connect();
  </script>
</body>
</html>
)rawliteral";

#endif
