#!/usr/bin/env python
import rospy
from std_msgs.msg import *

# from std_msgs.msg import Bool
# from std_msgs.msg import String
# from std_msgs.msg import Int8
# from std_msgs.msg import Float32


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
IS_SPEAKING = False

def main(responses):
    pub = rospy.Publisher('tocabi/emotion', Int64, queue_size=10)
    overlay_control_pub = rospy.Publisher('overlay_command', String, queue_size=5)
    pose_calibration_pub = rospy.Publisher('/tocabi/dg/avatar/pose_calibration_flag',Int8,queue_size=5)
    retargeting_sync_pub = rospy.Publisher('/tocabi/dg/upperbodymodecommand',Float32,queue_size=5)

    rospy.init_node('Emotion')
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
            cur_action = D.decide(IS_SPEAKING, "")
            print("Started Speaking!")
            if cur_action != prev_action:
                prev_action = cur_action

            prev_speaking_flag = cur_flag
            pub.publish(cur_action)
        
        if not result.alternatives:
            continue
        
        transcript = result.alternatives[0].transcript.lower()
        
        if result.is_final:
            
            if "close" in transcript:
                overlay_control_pub.publish("close")
            if "open" in transcript:
                overlay_control_pub.publish("open")
            if "right" in transcript:
                overlay_control_pub.publish("right")
            if "left" in transcript:
                overlay_control_pub.publish("left")
            if "up" in transcript:
                overlay_control_pub.publish("up")
            if "down" in transcript:
                overlay_control_pub.publish("down")
            if "front" in transcript:
                overlay_control_pub.publish("front")
            if "back" in transcript:
                overlay_control_pub.publish("back")
            if "opacity" in transcript:
                overlay_control_pub.publish(transcript)

            if "1" in transcript:
                pose_calibration_pub.publish(1)
            elif "2" in transcript:
                pose_calibration_pub.publish(2)
            elif "3" in transcript:
                pose_calibration_pub.publish(3)
            elif "4" in transcript:
                pose_calibration_pub.publish(4)
            elif "5" in transcript:
                pose_calibration_pub.publish(5)

            if "stop" in transcript:
                retargeting_sync_pub.publish(3)
            

            IS_SPEAKING = False
            # TODO: Modulize Action Part  
            cur_action = D.decide(IS_SPEAKING, transcript)
            print "Finished speaking. Recognized sentence: ", "'",transcript,"'"
            print "Classified Emotion Index: ", cur_action,"\n"

            if cur_action != prev_action:
                prev_action = cur_action

            prev_speaking_flag = -1
            pub.publish(cur_action)
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
    print("=================================")
    print("Configuration Set!")
    with mic_manager as stream:
            
        audio_generator = stream.generator()
        requests = (
            types.StreamingRecognizeRequest(audio_content=content)
            for content in audio_generator
        )
        print("=================================")
        print("Please speak anything to start...")
        responses = client.streaming_recognize(streaming_config, requests)
        main(responses)