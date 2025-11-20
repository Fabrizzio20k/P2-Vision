#!/usr/bin/env python3

import asyncio
import websockets
import json
import base64
import cv2
import numpy as np
import math
import gradio as gr
from threading import Thread
import time


latest_image = None
latest_lidar = None
latest_battery = None
connected = False
websocket_connection = None


def draw_lidar(ranges, width=600, height=600):
    img = np.zeros((height, width, 3), dtype=np.uint8)
    center_x = width // 2
    center_y = height // 2
    max_range = 3.0
    scale = min(width, height) // 2 / max_range

    cv2.circle(img, (center_x, center_y), 5, (0, 255, 0), -1)

    num_points = len(ranges)
    angle_increment = 2 * math.pi / num_points

    for i, r in enumerate(ranges):
        if r > 0.1 and r < max_range:
            angle = i * angle_increment
            x = int(center_x + r * scale * math.cos(angle))
            y = int(center_y - r * scale * math.sin(angle))
            cv2.circle(img, (x, y), 3, (0, 0, 255), -1)

    cv2.circle(img, (center_x, center_y), int(1.0 * scale), (100, 100, 100), 2)
    cv2.circle(img, (center_x, center_y), int(2.0 * scale), (100, 100, 100), 2)

    cv2.putText(img, "1m", (center_x + int(1.0 * scale) + 10, center_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(img, "2m", (center_x + int(2.0 * scale) + 10, center_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


async def send_velocity(linear, angular):
    global websocket_connection
    if websocket_connection:
        cmd = {
            'type': 'cmd_vel',
            'linear': linear,
            'angular': angular
        }
        try:
            await websocket_connection.send(json.dumps(cmd))
        except Exception as e:
            print(f"Error enviando velocidad: {e}")


async def receive_from_robot():
    global latest_image, latest_lidar, latest_battery, connected, websocket_connection
    uri = "ws://172.20.10.7:8765"

    while True:
        try:
            async with websockets.connect(uri) as websocket:
                websocket_connection = websocket
                connected = True
                print(f"✓ Conectado al TurtleBot4")
                while True:
                    message = await websocket.recv()
                    if not message.strip():
                        continue
                    data = json.loads(message)

                    if isinstance(data, dict):
                        if 'image' in data:
                            img_bytes = base64.b64decode(data['image']['data'])
                            img_array = np.frombuffer(
                                img_bytes, dtype=np.uint8)
                            frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                            if frame is not None:
                                latest_image = cv2.cvtColor(
                                    frame, cv2.COLOR_BGR2RGB)

                        if 'scan' in data:
                            latest_lidar = draw_lidar(data['scan']['ranges'])

                        if 'battery' in data:
                            latest_battery = data['battery']
                    else:
                        print(f"Mensaje inesperado: {data}")

        except Exception as e:
            connected = False
            websocket_connection = None
            print(f"Error: {e}. Reconectando en 2s...")
            await asyncio.sleep(2)


def start_websocket():
    asyncio.run(receive_from_robot())


def update_camera():
    if latest_image is not None:
        return latest_image
    return np.zeros((240, 320, 3), dtype=np.uint8)


def update_lidar():
    if latest_lidar is not None:
        return latest_lidar
    return np.zeros((600, 600, 3), dtype=np.uint8)


def get_status():
    if connected and latest_image is not None and latest_lidar is not None:
        return "🟢 Conectado - Recibiendo datos"
    elif connected:
        return "🟡 Conectado - Esperando datos..."
    else:
        return "🔴 Desconectado"


def get_battery_info():
    if latest_battery:
        percentage = latest_battery.get('percentage', 0)
        voltage = latest_battery.get('voltage', 0)
        current = latest_battery.get('current', 0)
        status = latest_battery.get('power_supply_status', 0)

        status_text = {0: "Desconocido", 1: "Cargando",
                       2: "Descargando", 3: "Sin cargar", 4: "Lleno"}
        status_str = status_text.get(status, "Desconocido")

        if percentage > 80:
            emoji = "🟢"
        elif percentage > 20:
            emoji = "🟡"
        else:
            emoji = "🔴"

        return f"{emoji} {percentage:.1f}% | {voltage:.2f}V | {current:.2f}A | {status_str}"
    return "⚪ Sin datos de batería"


def move_forward(linear_speed):
    asyncio.run(send_velocity(linear_speed, 0.0))
    return f"⬆️ Adelante: {linear_speed} m/s"


def move_backward(linear_speed):
    asyncio.run(send_velocity(-linear_speed, 0.0))
    return f"⬇️ Atrás: {linear_speed} m/s"


def turn_left(angular_speed):
    asyncio.run(send_velocity(0.0, angular_speed))
    return f"⬅️ Izquierda: {angular_speed} rad/s"


def turn_right(angular_speed):
    asyncio.run(send_velocity(0.0, -angular_speed))
    return f"➡️ Derecha: {angular_speed} rad/s"


def stop_robot():
    asyncio.run(send_velocity(0.0, 0.0))
    return "⏹️ Detenido"


ws_thread = Thread(target=start_websocket, daemon=True)
ws_thread.start()

keyboard_js = """
<script>
function handleKeyboard(e) {
    const key = e.key.toLowerCase();
    
    if (key === 'w') {
        document.getElementById('btn_forward').click();
        e.preventDefault();
    } else if (key === 's') {
        document.getElementById('btn_stop').click();
        e.preventDefault();
    } else if (key === 'a') {
        document.getElementById('btn_left').click();
        e.preventDefault();
    } else if (key === 'd') {
        document.getElementById('btn_right').click();
        e.preventDefault();
    } else if (key === 'x') {
        document.getElementById('btn_backward').click();
        e.preventDefault();
    }
}

document.addEventListener('keydown', handleKeyboard, false);
</script>
"""

with gr.Blocks(title="TurtleBot4 Monitor", theme=gr.themes.Soft(), head=keyboard_js) as demo:
    gr.Markdown("# 🤖 TurtleBot4 Live Monitor & Control")

    with gr.Row():
        status = gr.Textbox(label="Estado Conexión", value=get_status, every=1)
        battery_status = gr.Textbox(
            label="🔋 Batería", value=get_battery_info, every=1)

    with gr.Row():
        with gr.Column():
            gr.Markdown("### 📷 Cámara")
            camera_output = gr.Image(
                label="Vista en vivo", every=0.05, value=update_camera)

        with gr.Column():
            gr.Markdown("### 📡 LIDAR")
            lidar_output = gr.Image(
                label="Vista en vivo", every=0.05, value=update_lidar)

    gr.Markdown("### 🎮 Control del Robot")
    gr.Markdown(
        "**Teclado:** W=Adelante | A=Izquierda | S=Detener | D=Derecha | X=Atrás")

    with gr.Row():
        linear_speed = gr.Slider(
            0.1, 0.5, value=0.2, step=0.05, label="Velocidad Lineal (m/s)")
        angular_speed = gr.Slider(
            0.1, 2.0, value=0.5, step=0.1, label="Velocidad Angular (rad/s)")

    control_status = gr.Textbox(
        label="Estado del Control", value="⏹️ Detenido")

    with gr.Row():
        btn_forward = gr.Button("⬆️ Adelante (W)", elem_id="btn_forward")
        btn_backward = gr.Button("⬇️ Atrás (X)", elem_id="btn_backward")

    with gr.Row():
        btn_left = gr.Button("⬅️ Izquierda (A)", elem_id="btn_left")
        btn_stop = gr.Button(
            "⏹️ Detener (S)", elem_id="btn_stop", variant="stop")
        btn_right = gr.Button("➡️ Derecha (D)", elem_id="btn_right")

    btn_forward.click(move_forward, inputs=[
                      linear_speed], outputs=[control_status])
    btn_backward.click(move_backward, inputs=[
                       linear_speed], outputs=[control_status])
    btn_left.click(turn_left, inputs=[angular_speed], outputs=[control_status])
    btn_right.click(turn_right, inputs=[
                    angular_speed], outputs=[control_status])
    btn_stop.click(stop_robot, outputs=[control_status])


if __name__ == "__main__":
    print("🌐 Abriendo interfaz Gradio...")
    demo.launch(server_name="0.0.0.0", server_port=7860, share=False)
