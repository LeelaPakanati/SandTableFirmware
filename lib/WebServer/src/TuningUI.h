#pragma once

const char TUNING_UI_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Sisyphus Tuning</title>
    <style>
        :root {
            --bg-dark: #0f0f1a;
            --bg-card: rgba(30, 30, 50, 0.8);
            --bg-card-hover: rgba(40, 40, 65, 0.9);
            --accent: #c9a227;
            --accent-light: #e8c547;
            --accent-dim: rgba(201, 162, 39, 0.3);
            --text-primary: #f5f5f5;
            --text-secondary: #a0a0b0;
            --text-muted: #606070;
            --border: rgba(255, 255, 255, 0.1);
            --success: #4ade80;
            --warning: #fbbf24;
            --danger: #f87171;
        }

        * { margin: 0; padding: 0; box-sizing: border-box; }

        body {
            font-family: 'SF Pro Display', -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: var(--bg-dark);
            background-image:
                radial-gradient(ellipse at top, rgba(201, 162, 39, 0.15) 0%, transparent 50%),
                radial-gradient(ellipse at bottom, rgba(30, 30, 50, 0.5) 0%, transparent 50%);
            min-height: 100vh;
            padding: 20px;
            color: var(--text-primary);
        }

        .container { max-width: 800px; margin: 0 auto; }

        .header {
            text-align: center;
            margin-bottom: 32px;
        }

        .logo {
            font-size: 2.5em;
            font-weight: 300;
            letter-spacing: 8px;
            color: var(--text-primary);
            text-transform: uppercase;
            margin-bottom: 8px;
        }

        .logo span { color: var(--accent); }

        .tagline {
            font-size: 0.9em;
            color: var(--text-muted);
            letter-spacing: 2px;
        }

        .navbar {
            display: flex;
            justify-content: center;
            gap: 8px;
            margin-bottom: 32px;
            padding: 6px;
            background: var(--bg-card);
            border-radius: 50px;
            backdrop-filter: blur(10px);
            border: 1px solid var(--border);
            width: fit-content;
            margin-left: auto;
            margin-right: auto;
        }

        .nav-link {
            color: var(--text-secondary);
            text-decoration: none;
            padding: 10px 24px;
            border-radius: 50px;
            font-weight: 500;
            font-size: 0.9em;
            transition: all 0.3s ease;
        }

        .nav-link:hover {
            color: var(--text-primary);
            background: rgba(255, 255, 255, 0.05);
        }

        .nav-link.active {
            background: var(--accent);
            color: var(--bg-dark);
            font-weight: 600;
        }

        .card {
            background: var(--bg-card);
            border-radius: 20px;
            padding: 24px;
            margin-bottom: 20px;
            backdrop-filter: blur(10px);
            border: 1px solid var(--border);
        }

        .card-title {
            font-size: 0.8em;
            font-weight: 600;
            letter-spacing: 2px;
            text-transform: uppercase;
            color: var(--accent);
            margin-bottom: 20px;
        }

        .section-title {
            font-size: 0.75em;
            font-weight: 600;
            letter-spacing: 1px;
            text-transform: uppercase;
            color: var(--text-muted);
            margin-bottom: 16px;
            margin-top: 24px;
            padding-top: 16px;
            border-top: 1px solid var(--border);
        }

        .section-title:first-of-type {
            margin-top: 0;
            padding-top: 0;
            border-top: none;
        }

        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
            gap: 12px;
            margin-bottom: 16px;
        }

        .form-group {
            margin-bottom: 0;
        }

        label {
            display: block;
            margin-bottom: 6px;
            font-size: 0.8em;
            color: var(--text-secondary);
        }

        input[type="number"], select {
            width: 100%;
            padding: 12px;
            border: 1px solid var(--border);
            border-radius: 10px;
            font-size: 0.95em;
            background: #1a1a2e;
            color: var(--text-primary);
            transition: border-color 0.2s;
        }

        input[type="number"]:focus, select:focus {
            outline: none;
            border-color: var(--accent);
        }

        select option {
            background: #1a1a2e;
            color: #f5f5f5;
            padding: 12px;
        }

        button {
            width: 100%;
            padding: 14px;
            border: none;
            border-radius: 12px;
            font-size: 0.95em;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.3s ease;
            margin-bottom: 8px;
        }

        .btn-primary {
            background: var(--accent);
            color: var(--bg-dark);
        }

        .btn-primary:hover {
            background: var(--accent-light);
            transform: translateY(-2px);
            box-shadow: 0 8px 20px rgba(201, 162, 39, 0.3);
        }

        .btn-secondary {
            background: rgba(255, 255, 255, 0.1);
            color: var(--text-primary);
            border: 1px solid var(--border);
        }

        .btn-secondary:hover {
            background: rgba(255, 255, 255, 0.15);
        }

        .test-buttons {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 12px;
            margin-top: 16px;
        }

        .btn-test-theta {
            background: var(--success);
            color: #000;
        }

        .btn-test-theta:hover {
            background: #6ee7a0;
        }

        .btn-test-rho {
            background: var(--warning);
            color: #000;
        }

        .btn-test-rho:hover {
            background: #fcd34d;
        }

        .test-description {
            font-size: 0.85em;
            color: var(--text-muted);
            margin-bottom: 16px;
            line-height: 1.5;
        }

        ::-webkit-scrollbar { width: 6px; }
        ::-webkit-scrollbar-track { background: transparent; }
        ::-webkit-scrollbar-thumb { background: var(--text-muted); border-radius: 3px; }

        @media (max-width: 768px) {
            body { padding: 12px; }
            .logo { font-size: 1.8em; letter-spacing: 4px; }
            .grid { grid-template-columns: 1fr 1fr; }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <div class="logo">Sisy<span>phus</span></div>
            <div class="tagline">System Tuning</div>
        </div>

        <nav class="navbar">
            <a href="/" class="nav-link">Dashboard</a>
            <a href="/files" class="nav-link">Files</a>
            <a href="/tuning" class="nav-link active">Tuning</a>
        </nav>

        <div class="card">
            <div class="card-title">Motion Settings</div>
            <div class="grid">
                <div class="form-group">
                    <label>Rho Max Velocity (mm/s)</label>
                    <input type="number" id="tune-rMaxVelocity" step="0.1">
                </div>
                <div class="form-group">
                    <label>Rho Max Accel (mm/s²)</label>
                    <input type="number" id="tune-rMaxAccel" step="0.1">
                </div>
                <div class="form-group">
                    <label>Rho Max Jerk (mm/s³)</label>
                    <input type="number" id="tune-rMaxJerk" step="1">
                </div>
                <div class="form-group">
                    <label>Theta Max Velocity (rad/s)</label>
                    <input type="number" id="tune-tMaxVelocity" step="0.1">
                </div>
                <div class="form-group">
                    <label>Theta Max Accel (rad/s²)</label>
                    <input type="number" id="tune-tMaxAccel" step="0.1">
                </div>
                <div class="form-group">
                    <label>Theta Max Jerk (rad/s³)</label>
                    <input type="number" id="tune-tMaxJerk" step="1">
                </div>
            </div>
            <button class="btn-primary" id="btn-save-motion">Save Motion Settings</button>
        </div>

        <div class="card">
            <div class="card-title">Theta Driver</div>
            <div class="grid">
                <div class="form-group">
                    <label>Current (mA)</label>
                    <input type="number" id="tune-theta-current" step="50">
                </div>
                <div class="form-group">
                    <label>Toff</label>
                    <input type="number" id="tune-theta-toff" min="1" max="15">
                </div>
                <div class="form-group">
                    <label>Blank Time</label>
                    <select id="tune-theta-blankTime">
                        <option value="16">16</option>
                        <option value="24">24</option>
                        <option value="36">36</option>
                        <option value="54">54</option>
                    </select>
                </div>
                <div class="form-group">
                    <label>Mode</label>
                    <select id="tune-theta-spreadCycle">
                        <option value="false">StealthChop</option>
                        <option value="true">SpreadCycle</option>
                    </select>
                </div>
                <div class="form-group">
                    <label>PWM Freq</label>
                    <select id="tune-theta-pwmFreq">
                        <option value="0">2/1024</option>
                        <option value="1">2/683</option>
                        <option value="2">2/512</option>
                        <option value="3">2/410</option>
                    </select>
                </div>
                <div class="form-group">
                    <label>TPWMTHRS</label>
                    <input type="number" id="tune-theta-tpwmthrs">
                </div>
            </div>
            <button class="btn-primary" id="btn-save-theta">Save Theta Driver</button>
        </div>

        <div class="card">
            <div class="card-title">Rho Driver</div>
            <div class="grid">
                <div class="form-group">
                    <label>Current (mA)</label>
                    <input type="number" id="tune-rho-current" step="50">
                </div>
                <div class="form-group">
                    <label>Toff</label>
                    <input type="number" id="tune-rho-toff" min="1" max="15">
                </div>
                <div class="form-group">
                    <label>Blank Time</label>
                    <select id="tune-rho-blankTime">
                        <option value="16">16</option>
                        <option value="24">24</option>
                        <option value="36">36</option>
                        <option value="54">54</option>
                    </select>
                </div>
                <div class="form-group">
                    <label>Mode</label>
                    <select id="tune-rho-spreadCycle">
                        <option value="false">StealthChop</option>
                        <option value="true">SpreadCycle</option>
                    </select>
                </div>
                <div class="form-group">
                    <label>PWM Freq</label>
                    <select id="tune-rho-pwmFreq">
                        <option value="0">2/1024</option>
                        <option value="1">2/683</option>
                        <option value="2">2/512</option>
                        <option value="3">2/410</option>
                    </select>
                </div>
                <div class="form-group">
                    <label>TPWMTHRS</label>
                    <input type="number" id="tune-rho-tpwmthrs">
                </div>
            </div>
            <button class="btn-primary" id="btn-save-rho">Save Rho Driver</button>
        </div>

        <div class="card">
            <div class="card-title">Motor Tests</div>
            <p class="test-description">
                Tests ramp up to full speed, slow down, reverse direction, then perform quick random start/stops.
                Clear the table before testing.
            </p>
            <div class="test-buttons">
                <button class="btn-test-theta" id="btn-test-theta">Test Theta</button>
                <button class="btn-test-rho" id="btn-test-rho">Test Rho</button>
            </div>
        </div>
    </div>

    <script>
        const apiBase = '/api';

        async function loadSettings() {
            try {
                const response = await fetch(apiBase + '/tuning');
                const data = await response.json();

                document.getElementById('tune-rMaxVelocity').value = data.motion.rMaxVelocity;
                document.getElementById('tune-rMaxAccel').value = data.motion.rMaxAccel;
                document.getElementById('tune-rMaxJerk').value = data.motion.rMaxJerk;
                document.getElementById('tune-tMaxVelocity').value = data.motion.tMaxVelocity;
                document.getElementById('tune-tMaxAccel').value = data.motion.tMaxAccel;
                document.getElementById('tune-tMaxJerk').value = data.motion.tMaxJerk;

                document.getElementById('tune-theta-current').value = data.thetaDriver.current;
                document.getElementById('tune-theta-toff').value = data.thetaDriver.toff;
                document.getElementById('tune-theta-blankTime').value = data.thetaDriver.blankTime;
                document.getElementById('tune-theta-spreadCycle').value = data.thetaDriver.spreadCycle ? 'true' : 'false';
                document.getElementById('tune-theta-pwmFreq').value = data.thetaDriver.pwmFreq;
                document.getElementById('tune-theta-tpwmthrs').value = data.thetaDriver.tpwmthrs;

                document.getElementById('tune-rho-current').value = data.rhoDriver.current;
                document.getElementById('tune-rho-toff').value = data.rhoDriver.toff;
                document.getElementById('tune-rho-blankTime').value = data.rhoDriver.blankTime;
                document.getElementById('tune-rho-spreadCycle').value = data.rhoDriver.spreadCycle ? 'true' : 'false';
                document.getElementById('tune-rho-pwmFreq').value = data.rhoDriver.pwmFreq;
                document.getElementById('tune-rho-tpwmthrs').value = data.rhoDriver.tpwmthrs;
            } catch (err) {
                console.error('Failed to load settings:', err);
            }
        }

        async function saveMotion() {
            const formData = new FormData();
            formData.append('rMaxVelocity', document.getElementById('tune-rMaxVelocity').value);
            formData.append('rMaxAccel', document.getElementById('tune-rMaxAccel').value);
            formData.append('rMaxJerk', document.getElementById('tune-rMaxJerk').value);
            formData.append('tMaxVelocity', document.getElementById('tune-tMaxVelocity').value);
            formData.append('tMaxAccel', document.getElementById('tune-tMaxAccel').value);
            formData.append('tMaxJerk', document.getElementById('tune-tMaxJerk').value);
            await fetch(apiBase + '/tuning/motion', { method: 'POST', body: formData });
            alert('Motion settings saved');
        }

        async function saveTheta() {
            const formData = new FormData();
            formData.append('current', document.getElementById('tune-theta-current').value);
            formData.append('toff', document.getElementById('tune-theta-toff').value);
            formData.append('blankTime', document.getElementById('tune-theta-blankTime').value);
            formData.append('spreadCycle', document.getElementById('tune-theta-spreadCycle').value);
            formData.append('pwmFreq', document.getElementById('tune-theta-pwmFreq').value);
            formData.append('tpwmthrs', document.getElementById('tune-theta-tpwmthrs').value);
            await fetch(apiBase + '/tuning/theta', { method: 'POST', body: formData });
            alert('Theta driver saved');
        }

        async function saveRho() {
            const formData = new FormData();
            formData.append('current', document.getElementById('tune-rho-current').value);
            formData.append('toff', document.getElementById('tune-rho-toff').value);
            formData.append('blankTime', document.getElementById('tune-rho-blankTime').value);
            formData.append('spreadCycle', document.getElementById('tune-rho-spreadCycle').value);
            formData.append('pwmFreq', document.getElementById('tune-rho-pwmFreq').value);
            formData.append('tpwmthrs', document.getElementById('tune-rho-tpwmthrs').value);
            await fetch(apiBase + '/tuning/rho', { method: 'POST', body: formData });
            alert('Rho driver saved');
        }

        async function testTheta() {
            const btn = document.getElementById('btn-test-theta');
            btn.disabled = true;
            btn.textContent = 'Testing...';
            try {
                await fetch(apiBase + '/tuning/test/theta', { method: 'POST' });
            } catch (err) {
                console.error('Test failed:', err);
            }
            btn.disabled = false;
            btn.textContent = 'Test Theta';
        }

        async function testRho() {
            const btn = document.getElementById('btn-test-rho');
            btn.disabled = true;
            btn.textContent = 'Testing...';
            try {
                await fetch(apiBase + '/tuning/test/rho', { method: 'POST' });
            } catch (err) {
                console.error('Test failed:', err);
            }
            btn.disabled = false;
            btn.textContent = 'Test Rho';
        }

        document.getElementById('btn-save-motion').addEventListener('click', saveMotion);
        document.getElementById('btn-save-theta').addEventListener('click', saveTheta);
        document.getElementById('btn-save-rho').addEventListener('click', saveRho);
        document.getElementById('btn-test-theta').addEventListener('click', testTheta);
        document.getElementById('btn-test-rho').addEventListener('click', testRho);

        window.onload = loadSettings;
    </script>
</body>
</html>
)rawliteral";
