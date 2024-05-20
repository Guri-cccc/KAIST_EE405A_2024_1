#!/usr/bin/env python3
import base64
import pkg_resources
import rospy
import openai
import rospkg
import base64
import os
import re
import time

from std_msgs.msg import String
from llm_msgs.srv import CallImage, CallImageRequest
from llm_msgs.msg import task_plan

class VisualLanguageAPI():
    def __init__(self):
        rospy.init_node('vision_language')
        self.package_path = rospkg.RosPack().get_path('llm')
        rospy.Subscriber('/vision_query', String, self.query_callback, queue_size = 1)
        
        self.task_planner = rospy.Publisher('/task_plan', task_plan, queue_size=1)
        self.api_key = self.open_api_key()
        openai.api_key = self.api_key  
        self.model = "gpt-4-vision-preview" 
        self.max_tokens = 3000
        
        self.query = ""
        self.prompt = self.apply_prompt()
        self.image_path = self.package_path + '/images/'
        
        

    def open_api_key(self):
        with open(self.package_path + "/api_key/" + "api_key.txt") as file:
            api_key = file.read().strip()
        return api_key

    def apply_prompt(self):
        with open(self.package_path + "/api_key/" + "prompt.txt") as file:
            prompt = file.read().strip()
        return prompt
        
    def query_callback(self, msg):
        self.query = msg.data
        print("Query Deliver and Image Saved ->", self.call_image_saved_service())
        if self.call_image_saved_service():
            print("Processing.........")
            self.vision_input = self.image_path + 'gpt_vision.jpg' 
            try:
                with open(self.vision_input, "rb") as image_file:
                    response = self.visual_language_model_from_image(self.vision_input)
                    print(response)
                    self.extract_tuple(response)
                    
            except IOError as e:
                print("Error opening image file: %s" % e)
        else:
            print("Image not saved yet or service call failed...")
        
    def encode_image_file(self, image_path):
        with open(image_path, "rb") as image_file:
            image_content = image_file.read()
            encode = base64.b64encode(image_content).decode('utf-8')
            return encode
        
    def visual_language_model_from_image(self, base64_image):
        image_url = f"data:image/jpeg;base64,{self.encode_image_file(base64_image)}"
        response = openai.ChatCompletion.create(
            model = self.model, 
            messages = [{"role": "user",
                         "content": [{"type": "text", 
                                      "text": self.prompt + self.query},  
                        {"type": "image_url",
                         "image_url": {"url": image_url,
                                       "detail": "high"}}],}],
            max_tokens = self.max_tokens)
        
        return response["choices"][0]["message"]["content"]
    
    def call_image_saved_service(self):
        rospy.wait_for_service('image_saved')
        try:
            image_saved = rospy.ServiceProxy('image_saved', CallImage)
            req = CallImageRequest(saved = True)
            resp = image_saved(req)
            if resp.success:
                file_path = self.image_path + 'gpt_vision.jpg'
                if os.path.exists(file_path):
                    time.sleep(1)
                    return True

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return False
    
    def extract_tuple(self, response):
        answer = response
        ### TODO 2 ###
        ### Extract each result from Mode, Parameter, and Value ###
        ### Hint: Publsih each result using self.task_planner ###
            
            
            
def main():
    print("Current version of openai == ", pkg_resources.get_distribution("openai").version)
    VisualLanguageAPI()
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

# Main
if __name__ == '__main__':
    main()       