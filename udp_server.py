import socket
import json

PC_PORT = 7070

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
    s.bind(("192.168.1.109", PC_PORT))
    print(f"Aguardando telemetria UDP na porta {PC_PORT}...")

    while True:
        data, addr = s.recvfrom(2048)
        msg = data.decode(errors="ignore").strip()

        print(f"[{addr}] {msg}")

        try:
            print("JSON:", json.loads(msg))
        except json.JSONDecodeError:
            print("Mensagem recebida não é JSON válido")
