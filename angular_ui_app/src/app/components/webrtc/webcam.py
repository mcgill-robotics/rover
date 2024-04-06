import argparse
import asyncio
import json
import logging
import os
import platform
import ssl

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaPlayer

ROOT = os.path.dirname(__file__)
HOST_IP = os.getenv('HOST_IP', "0.0.0.0")

pcs = set() #set of all pc

logging.basicConfig(level=logging.DEBUG) #enable logging
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

    # open media source
    # if args.play_from:
    #     player = MediaPlayer(args.play_from)
    # else:
    #     options = {"framerate": "20", "video_size": "640x360"}
    #     if platform.system() == "Darwin": # macOS
    #         player = MediaPlayer("default:none", format="avfoundation", options=options)
    #     else:
    #         # player = MediaPlayer("/dev/video0", format="v4l2", options=options)
    #         player = MediaPlayer(f'/dev/video{id}', format="v4l2", options=options)

    options = {"framerate": "20", "video_size": "640x360"}
    if platform.system() == "Darwin": # macOS
        player = MediaPlayer("default:none", format="avfoundation", options=options)
    else:
        # player = MediaPlayer("/dev/video0", format="v4l2", options=options)
        player = MediaPlayer(f'/dev/video{id}', format="v4l2", options=options)

    await pc.setRemoteDescription(offer)
    for t in pc.getTransceivers():
        # if t.kind == "audio" and player.audio:
        #     pc.addTrack(player.audio)
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
    print("returning")

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
    parser = argparse.ArgumentParser(description="WebRTC webcam demo")
    # parser.add_argument("--cert-file", help="SSL certificate file (for HTTPS)")
    # parser.add_argument("--key-file", help="SSL key file (for HTTPS)")
    # parser.add_argument("--play-from", help="Read the media from a file and sent it."),
    parser.add_argument(
        "--host", default=HOST_IP, help="Host for HTTP server (default: 0.0.0.0)" #can choose which IP to us
        
            )
    parser.add_argument(
        "--port", type=int, default=8080, help="Port for HTTP server (default: 8080)"
    )
    parser.add_argument("--verbose", "-v", action="count")
    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)

    # if args.cert_file:
    #     ssl_context = ssl.SSLContext()
    #     ssl_context.load_cert_chain(args.cert_file, args.key_file)
    # else:
        # ssl_context = None
    ssl_context = None

    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    app.router.add_options("/offer", handle_options) 
    
    app.router.add_post("/offer", offer)
    web.run_app(app, host=args.host, port=args.port, ssl_context=ssl_context)