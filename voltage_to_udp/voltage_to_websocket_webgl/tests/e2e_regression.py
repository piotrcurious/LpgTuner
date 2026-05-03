import os
import subprocess
import time
from playwright.sync_api import sync_playwright

def run_regression():
    # 1. Start Mock Server
    with open('/home/jules/verification/regression_mock.log', 'w') as f:
        mock = subprocess.Popen(['python3', '/home/jules/self_created_tools/mock_esp32_server.py'],
                              stdout=f, stderr=f)
    time.sleep(2)

    html_content = ""
    with open('voltage_to_udp/voltage_to_websocket_webgl/frontend.h', 'r') as f:
        content = f.read()
        start = content.find('R"rawliteral(') + len('R"rawliteral(')
        end = content.find(')rawliteral"')
        html_content = content[start:end]

    html_content = html_content.replace(
        "const ws = new WebSocket(protocol + '//' + location.host + '/ws');",
        "const ws = new WebSocket('ws://127.0.0.1:8080');"
    )

    temp_html = '/home/jules/verification/regression.html'
    with open(temp_html, 'w') as f:
        f.write(html_content)

    try:
        with sync_playwright() as p:
            browser = p.chromium.launch(headless=True)
            page = browser.new_page()
            page.goto(f'file://{temp_html}')

            print("Verifying Initial Connection...")
            page.wait_for_selector("text=LINK: UP", timeout=5000)

            print("Testing Mode Switch...")
            page.click("#btnMode1")
            page.wait_for_selector("text=[CAM MODE]")
            page.screenshot(path='/home/jules/verification/reg_cam_mode.png')

            print("Testing Simulation Toggle...")
            page.click("#btnSim")
            time.sleep(1)
            page.screenshot(path='/home/jules/verification/reg_sim_off.png')

            print("Testing Noise Slider...")
            # Use page.fill or dispatch event if it's a slider
            page.evaluate("document.getElementById('ctrl-noise').value = 50; updateNoise();")
            time.sleep(1)
            page.screenshot(path='/home/jules/verification/reg_noise_50.png')

            print("Regression Tests Completed Successfully.")
            browser.close()
    except Exception as e:
        print(f"Regression Failed: {e}")
    finally:
        mock.terminate()

if __name__ == "__main__":
    os.makedirs('/home/jules/verification', exist_ok=True)
    run_regression()
