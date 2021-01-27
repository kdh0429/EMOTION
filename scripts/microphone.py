from __future__ import division

import re
import sys
import time
import pyaudio

from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types

from six.moves import queue

RATE = 16000
CHUNK = int(RATE / 10)  # 100ms

def get_current_time():
    """Return Current Time in MS."""

    return int(round(time.time() * 1000))

class MicrophoneStream(object):
    def __init__(self, rate, chunk):
        self._rate = rate
        self._chunk = chunk
        
        self._buff = queue.Queue()
        self.closed = True
        # self.audio_input = []
        # self.start_time = get_current_time()
        # self.result_end_time = 0
        # self.final_request_end_time = 0
        # self.is_final_end_time = 0
        # self.bridging_offset = 0
        # self.last_audio_input = []
        # self.last_transcript_was_final = False
        # self.new_stream = True

    def __enter__(self):
        self._audio_interface = pyaudio.PyAudio()
        self._audio_stream = self._audio_interface.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self._rate,
            input=True,
            frames_per_buffer=self._chunk,
            stream_callback=self._fill_buffer,
        )
        self.closed = False
        return self

    def __exit__(self, type, value, traceback):
        self._audio_stream.stop_stream()
        self._audio_stream.close()
        self.closed = True
        self._buff.put(None)
        self._audio_interface.terminate()

    def _fill_buffer(self, in_data, frame_count, time_info, status_flags):
        """Continuously collect data from the audio stream, into the buffer."""
        self._buff.put(in_data)
        return None, pyaudio.paContinue

    def generator(self):
        while not self.closed:

            chunk = self._buff.get()
            
            if chunk is None:
                return
            data = [chunk]

            while True:
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                    
                except queue.Empty:
                    break

            yield b"".join(data)

# Not in Use
def listen_print_loop(responses):
    prev_flag = -1
    
    for response in responses:
        
        if not response.results:
            continue

        result = response.results[0]
        cur_flag = 0 if result.is_final == False else 1
        if prev_flag != cur_flag:
            if result.is_final:
                print("!!!End!!!!")
            else:
                print("!!!Start Speaking!!!")
            prev_flag = cur_flag
        else:
            continue
        
        
        if not result.alternatives:
            continue
        
        transcript = result.alternatives[0].transcript  # Transcription of the top alternatives

        if result.is_final:
            #print(transcript + overwrite_chars)
            print(transcript)
            prev_flag = -1

            if re.search(r"\b(exit|quit)\b", transcript, re.I):
                print("Exiting..")
                break


def main():
    language_code = "en-US"
    client = speech.SpeechClient()

    config = types.RecognitionConfig(
    encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
    sample_rate_hertz=RATE,
    language_code=language_code,
    )

    streaming_config = speech.StreamingRecognitionConfig(
        config=config, interim_results=True
    )

    with MicrophoneStream(RATE, CHUNK) as stream:
        audio_generator = stream.generator()
        requests = (
            speech.StreamingRecognizeRequest(audio_content=content)
            for content in audio_generator
        )

        responses = client.streaming_recognize(streaming_config, requests)
        # Now, put the transcription responses to use.
        listen_print_loop(responses)

if __name__ == "__main__":
    main()