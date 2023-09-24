import os
import sys
from flask import Flask, send_from_directory, make_response

app = Flask(__name__)

def serve_file(directory_path, filename):
    response = make_response(send_from_directory(directory_path, filename))
    # Allow all origins
    response.headers['Access-Control-Allow-Origin'] = '*'
    return response

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: python program.py [port] [directory]")
        sys.exit(1)

    try:
        port = int(sys.argv[1])
        directory_path = sys.argv[2]
    except ValueError:
        print("Port must be an integer")
        sys.exit(1)

    if not os.path.exists(directory_path):
        print(f"The directory '{directory_path}' does not exist.")
        sys.exit(1)

    app.add_url_rule('/files/<filename>', 'serve_file', lambda filename: serve_file(directory_path, filename))
    app.run(host='0.0.0.0', port=port)
