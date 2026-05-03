import os
import subprocess
import time
from playwright.sync_api import sync_playwright

def run_regression():
    # Kill any existing mock server
    subprocess.run(['fuser', '-k', '8080/tcp'], capture_output=True)

    # 1. Start Mock Server
    os.makedirs('/home/jules/verification', exist_ok=True)
    with open('/home/jules/verification/regression_mock.log', 'w') as f:
        mock = subprocess.Popen(['/home/jules/.pyenv/versions/3.12.13/bin/python3', '/home/jules/self_created_tools/mock_esp32_server.py'],
                              stdout=f, stderr=f)
    time.sleep(3)

    html_content = ""
    with open('voltage_to_udp/voltage_to_websocket_webgl/frontend.h', 'r') as f:
        content = f.read()
        start = content.find('R"rawliteral(') + len('R"rawliteral(')
        end = content.find(')rawliteral"')
        html_content = content[start:end]

    # Correct replacement
    old_line = "ws = new WebSocket(protocol + '//' + location.host + '/ws');"
    new_line = "ws = new WebSocket('ws://127.0.0.1:8080');"
    if old_line in html_content:
        print("Found WebSocket line, replacing...")
        html_content = html_content.replace(old_line, new_line)
    else:
        print("WebSocket line NOT found! Check frontend.h")

    temp_html = '/home/jules/verification/regression.html'
    with open(temp_html, 'w') as f:
        f.write(html_content)

    try:
        with sync_playwright() as p:
            browser = p.chromium.launch(headless=True)
            page = browser.new_page()
            page.on('console', lambda msg: print(f'BROWSER {msg.type}: {msg.text}'))
            page.on('pageerror', lambda err: print(f'BROWSER ERROR: {err}'))
            page.goto(f'file://{temp_html}')

            print("Verifying Initial Connection...")
            time.sleep(10)

            stats_text = page.inner_text("#stats")
            print(f"Stats Overlay Text: {stats_text}")

            page.screenshot(path='/home/jules/verification/improved_ui.png')

            browser.close()
    except Exception as e:
        print(f"Regression Failed: {e}")
    finally:
        mock.terminate()

if __name__ == "__main__":
    run_regression()
