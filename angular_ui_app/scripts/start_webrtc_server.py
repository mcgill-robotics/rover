import asyncio
import json

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaPlayer

pcs = set() #set of all pc

# logging.basicConfig(level=logging.DEBUG) #enable logging
async def offer(request):
    print(request)
    id = request.rel_url.query["id"]

    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("iceconnectionstatechange")
    async def on_iceconnectionstatechange():
        print("ICE connection state is %s" % pc.iceConnectionState)
        if pc.iceConnectionState == "failed":
            await pc.close()
            pcs.discard(pc)

    options = {"framerate": "20", "video_size": "640x480"}
    player = MediaPlayer(f'/dev/video{id}', format="v4l2", options=options)

    await pc.setRemoteDescription(offer)
    for t in pc.getTransceivers():
        if t.kind == "video" and player.video:
            pc.addTrack(player.video)

    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    response = web.Response(
        content_type="application/json",
        text=json.dumps(
            {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
        ),
    )

    # Add CORS headers
    # response.headers['Access-Control-Allow-Origin'] = 'http://127.0.0.1:4200'  # Replace '*' with your specific allowed origins if necessary
    response.headers['Access-Control-Allow-Origin'] = '*'  # Replace '*' with your specific allowed origins if necessary
    response.headers['Access-Control-Allow-Methods'] = 'POST, OPTIONS, GET'
    response.headers['Access-Control-Allow-Headers'] = 'Content-Type'

    return response


async def handle_options(request):
    # Get the requested headers from the request
    requested_headers = request.headers.get('Access-Control-Request-Headers', '')

    # Respond to preflight requests
    return web.Response(
        status=200,
        headers={
            'Access-Control-Allow-Origin': '*',
            'Access-Control-Allow-Methods': 'POST, OPTIONS',
            'Access-Control-Allow-Headers': requested_headers,  # Use the requested headers in the response
        }
    )

async def on_shutdown(app):
    # close peer connections
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()

if __name__ == "__main__":
    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    app.router.add_options("/offer", handle_options)

    app.router.add_post("/offer", offer)
    web.run_app(app, host="0.0.0.0", port=8081, ssl_context=None)