#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64

from decision import Decision
from microphone import MicrophoneStream
from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types
import time
import re


LANGUAGE = "en-US"
RATE = 16000
CHUNK = int(RATE / 10)  # Unit: 100ms
STREAMING_LIMIT = 240000  # 4 min.
PORT = 'COM3'
BAUDRATE = 57600
MIN_DURATION = 0.1  # For Facial Expression (Unit: 1sec.)
IS_SPEAKING = False

def get_current_time():
    # Return current time in ms

    return int(round(time.time() * 1000))

def main(responses):
    pub = rospy.Publisher('tocabi/emotion', Int64, queue_size=10)
    rospy.init_node('Emotion', anonymous=True)

    prev_speaking_flag = -1
    prev_action = 1
    cur_action = 1
    D = Decision()
    for response in responses:
        
        if not response.results:
            continue

        result = response.results[0]
        
        cur_flag = 0 if result.is_final == False else 1
        if prev_speaking_flag != cur_flag and not result.is_final:
            # TODO: Modulize Action Part 
            IS_SPEAKING = True
            print("!!!Start Speaking!!!")
            cur_action = D.decide(IS_SPEAKING, "")
            if cur_action != prev_action:
                print("[FINAL ACTION]", cur_action)
                prev_action = cur_action

            pub.publish(cur_action)
            prev_speaking_flag = cur_flag
        
        if not result.alternatives:
            continue
        
        transcript = result.alternatives[0].transcript 
        
        if result.is_final:
            print(transcript, "\n!!!Speaking End!!!!")
            IS_SPEAKING = False
            # TODO: Modulize Action Part  
            cur_action = D.decide(IS_SPEAKING, transcript)
            if cur_action != prev_action:
                print("[FINAL ACTION]", cur_action)
                prev_action = cur_action

            pub.publish(cur_action)
            prev_speaking_flag = -1

            # Exit recognition if any of the transcribed phrases could be one of ["exit", "quit"]
            if re.search(r"\b(exit|quit)\b", transcript, re.I):
                print("Exiting..")
                break


if __name__ == "__main__":
    
    # STT Configuration
    client = speech.SpeechClient()
    config = types.RecognitionConfig(
    encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
    sample_rate_hertz=RATE,
    language_code=LANGUAGE,
    )

    mic_manager = MicrophoneStream(RATE, CHUNK)
    streaming_config = types.StreamingRecognitionConfig(
        config=config, interim_results=True
    )
    
    # Streaming STT
    with mic_manager as stream:
            
        audio_generator = stream.generator()
        
        requests = (
            types.StreamingRecognizeRequest(audio_content=content)
            for content in audio_generator
        )

        responses = client.streaming_recognize(streaming_config, requests)
        
        main(responses)
