from aiohttp import web
import socketio
import zmq
import argparse

parser = argparse.ArgumentParser(description='ZMQ Remote API bridge for socket.io clients.')
parser.add_argument('--host', '-H', type=str, default='localhost',
                    help='hostname to connect to (defaults to localhost)')
parser.add_argument('--port', '-P', type=int, default=23000,
                    help='port to connect to (defaults to 23000)')
parser.add_argument('--verbose', '-V', action='store_true',
                    help='port to connect to (defaults to 23000)')
args = parser.parse_args()

sio = socketio.AsyncServer()
app = web.Application()
sio.attach(app)

context = zmq.Context()
socket = context.socket(zmq.REQ)
addr = f'tcp://{args.host}:{args.port}'
print(f'======== Connecting to ZMQ socket {addr} ========')
socket.connect(addr)
socket.setsockopt(zmq.LINGER, 0)

async def index(request):
    with open('index.html') as f:
        return web.Response(text=f.read(), content_type='text/html')

@sio.on('message')
async def handler(sid, message):
    if args.verbose:
        print(f'> {message}')
    socket.send_json(message)
    rep = socket.recv_json()
    if args.verbose:
        print(f'< {rep}')
    return rep

app.router.add_get('/', index)

if __name__ == '__main__':
    web.run_app(app)

    socket.close()
