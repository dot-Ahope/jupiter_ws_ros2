#!/usr/bin/env python3
import requests
import json

def send_command(command, value):
    url = 'http://localhost:22001/command'
    data = {'command': command, 'value': value}
    headers = {'Content-Type': 'application/json'}
    
    try:
        response = requests.post(url, json=data, headers=headers)
        if response.status_code == 200:
            result = response.json()
            print(f"성공: {result['command']} 실행됨")
        else:
            print(f"오류: {response.status_code}")
    except Exception as e:
        print(f"연결 오류: {e}")

if __name__ == '__main__':
    send_command('rot', 60)
    
    #print("\n샘플: rot 90 전송")
    #send_command('rot', 90)
    
    #print("\n샘플: vel 1 전송")
    #send_command('vel', 1)
