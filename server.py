#!/usr/bin/env python3
from flask import Flask, request, jsonify
from flask_cors import CORS
import subprocess
import json
import os

app = Flask(__name__)
CORS(app)  # 모든 라우트에 CORS 적용

@app.route('/command', methods=['POST'])
def handle_command():
    try:
        data = request.get_json()
        if not data or 'command' not in data or 'value' not in data:
            return jsonify({"error": "Invalid JSON format. Expected {'command': 'move', 'value': 3}"}), 400
        
        command = data['command']
        value = data['value']
        
        # Validate command
        valid_commands = ['move', 'rot', 'vel']
        if command not in valid_commands:
            return jsonify({"error": f"Invalid command. Valid: {valid_commands}"}), 400
        
        # Construct the command to run
        cmd = ['python3', 'rover.py', command, str(value)]
        
        # Execute the command from the same directory as this script
        script_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Use bash to source ROS 2 environment and run the command
        ros_cmd = f'source /opt/ros/humble/setup.bash && source {script_dir}/install/setup.bash && python3 rover.py {command} {value}'
        cmd = ['bash', '-c', ros_cmd]
        
        result = subprocess.run(cmd, capture_output=True, text=True, cwd=script_dir)
        
        # Return the output
        return jsonify({
            "status": "success",
            "command": cmd,
            "stdout": result.stdout,
            "stderr": result.stderr,
            "returncode": result.returncode
        })
    
    except Exception as e:
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=22001, debug=True)
