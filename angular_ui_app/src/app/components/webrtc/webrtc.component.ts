import { Component } from '@angular/core';
import { ViewChild, ElementRef } from '@angular/core';

@Component({
  selector: 'app-webrtc',
  templateUrl: './webrtc.component.html',
  styleUrls: ['./webrtc.component.scss']
})
export class WebrtcComponent {
  pc: RTCPeerConnection | null = null;
  host_ip: string = "127.0.0.1";
  device_id: number = 0;
  error: unknown;

  @ViewChild('videoElement') videoElement: ElementRef;

  constructor() { }

  async negotiate() {
    try {
      this.pc?.addTransceiver('video', { direction: 'recvonly' });

      const offer = await this.pc?.createOffer();
      await this.pc?.setLocalDescription(offer);

      // Wait for ICE gathering to complete
      await new Promise<void>((resolve) => {
        if (this.pc?.iceGatheringState === 'complete') {
          resolve();
        } else {
          const checkState = () => {
            if (this.pc?.iceGatheringState === 'complete') {
              this.pc?.removeEventListener('icegatheringstatechange', checkState);
              resolve();
            }
          };
          this.pc?.addEventListener('icegatheringstatechange', checkState);
        }
      });


      // console.log("before fetch");
      let pd = await JSON.stringify({ sdp: this.pc?.localDescription?.sdp, type: this.pc?.localDescription?.type });
      console.log(pd);
      const headers = new Headers();
      headers.append('Content-Type', 'application/json');
      headers.append('Access-Control-Allow-Origin', '*');
      const response = await fetch(`http://${this.host_ip}:8080/offer?id=${this.device_id}`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Access-Control-Allow-Origin': '*',
        },
        body: JSON.stringify({ sdp: this.pc?.localDescription?.sdp, type: this.pc?.localDescription?.type })
      });
      console.log("after fetch and response is: ", response);
      const answer = await response.json();
      console.log("after response and answer is: ", answer);
      await this.pc?.setRemoteDescription(answer);
    } catch (error) {
      console.error('Error in negotiate:', error); 
      this.error = error;
      // Handle error as needed
    }
  }



  async start() {
    const config = { sdpSemantics: 'unified-plan', iceServers: [] as RTCIceServer[] }; // Define iceServers as an empty array initially
    if ((document.getElementById('use-stun') as HTMLInputElement).checked) {
      config.iceServers = [{ urls: ['stun:stun.l.google.com:19302'] }];
    }
    this.pc = new RTCPeerConnection(config);

    this.pc.addEventListener('track', (evt) => {
      if (evt.track.kind === 'video') {
        if (this.videoElement.nativeElement) {
          this.videoElement.nativeElement.srcObject = evt.streams[0];
        } else {
          console.error("Video element not found");
          this.error = "Video element not found";
        }
      }
    });

    await this.negotiate();
  }

  stop() {
    setTimeout(() => {
      this.pc?.close();
      // console.log(this.pc);
      this.pc = null; //set to null after closing
    }, 500);

  }
}