import http.server
import socketserver

PORT = 8080

handler = http.server.SimpleHTTPRequestHandler

print("python webserver load")
with socketserver.TCPServer(("", PORT), handler) as httpd:
    print("Server started at localhost:" + str(PORT))
    httpd.serve_forever()