from aiohttp import web
import socketio
import zmq

verbose = False

sio = socketio.AsyncServer()
app = web.Application()
sio.attach(app)

context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect('tcp://localhost:23000')

async def index(request):
    with open('index.html') as f:
        return web.Response(text=f.read(), content_type='text/html')

@sio.on('message')
async def handler(sid, message):
    if verbose:
        print('Received: ' + message)
    socket.send_string(message)
    rep = socket.recv_string()
    if verbose:
        print('Sending: ' + rep)
    return rep

app.router.add_get('/', index)

if __name__ == '__main__':
    web.run_app(app)
