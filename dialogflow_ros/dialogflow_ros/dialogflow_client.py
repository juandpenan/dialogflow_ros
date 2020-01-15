# Copyright 2019 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Dialogflow
import dialogflow_v2
from dialogflow_v2.types import InputAudioConfig, \
    OutputAudioConfig, QueryInput, QueryParameters, \
    StreamingDetectIntentRequest, TextInput
from dialogflow_v2.gapic.enums import AudioEncoding, OutputAudioEncoding
import google.api_core.exceptions
import google.auth
# utils
from .MicrophoneStream import MicrophoneStream
from . import utils
# Python
import pyaudio
import os
import time
from uuid import uuid4
from yaml import load, YAMLError
# ROS2
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String
from std_srvs.srv import Empty

from dialogflow_ros_msgs.msg import DialogflowRequest, DialogflowResult


class DialogflowClient(Node):
  def __init__(self, last_contexts=None):
    """Initialize all params and load data."""
    """ Constants and params """
    super().__init__('dialogflow_client')
    self.CHUNK = 4096
    self.FORMAT = pyaudio.paInt16
    self.CHANNELS = 1
    self.RATE = 16000
    parameters = [
      ('play_audio', '', ParameterDescriptor()),
      ('debug', '', ParameterDescriptor()),
      ('default_language', 'en-US', ParameterDescriptor()),
      ('results_topic', '/dialogflow_client/results', ParameterDescriptor()),
      ('requests_topic', '', ParameterDescriptor()),
      ('context_file', '', ParameterDescriptor())
    ]

    self.declare_parameters('', parameters)
    self.PLAY_AUDIO = self.get_parameter_or('play_audio',
      Parameter('play_audio', type_ = Parameter.Type.BOOL, value = True))._value
    self.DEBUG = self.get_parameter_or('debug', 
      Parameter('debug', type_ = Parameter.Type.BOOL, value = False))._value
    self._language_code = self.get_parameter_or('default_language', 
      Parameter('default_language', type_ = Parameter.Type.STRING, value = ''))._value
    results_topic = self.get_parameter_or('results_topic', 
      Parameter('results_topic', type_ = Parameter.Type.STRING, value = ""))._value
    requests_topic = self.get_parameter_or('requests_topic', 
      Parameter('requests_topic', type_ = Parameter.Type.STRING, value = "/dialogflow_client/requests"))._value
    context_file = self.get_parameter_or('context_file', 
      Parameter('context_file', type_ = Parameter.Type.STRING, value = ''))._value

    if os.environ['GOOGLE_APPLICATION_CREDENTIALS'] == '':
      self.get_logger().error('Missing credentials file. Set GOOGLE_APPLICATION_CREDENTIALS env var.')
      raise ValueError('Missing credentials file. Set GOOGLE_APPLICATION_CREDENTIALS env var.')
    
    self.credentials, project_id = google.auth.default()

    """ Dialogflow setup """
    if context_file:
      with open(context_file, 'r') as f:
        try:
          self._phrase_hints = load(f)
        except YAMLError:
          self.get_logger().warn("DF_CLIENT: Unable to open phrase hints yaml file!")
          self._phrase_hints = []
    else:
      self._phrase_hints = []

    # Dialogflow params
    session_id = str(uuid4())  # Random
    self.last_contexts = last_contexts if last_contexts else []
    # DF Audio Setup
    audio_encoding = AudioEncoding.AUDIO_ENCODING_LINEAR_16
    self._audio_config = InputAudioConfig(
      audio_encoding = audio_encoding,
      language_code = self._language_code,
      sample_rate_hertz = self.RATE,
      phrase_hints = self._phrase_hints)
    self._output_audio_config = OutputAudioConfig(
      audio_encoding = OutputAudioEncoding.OUTPUT_AUDIO_ENCODING_LINEAR_16
    )
    # Create a session
    self._session_cli = dialogflow_v2.SessionsClient(credentials=self.credentials)
    self._session = self._session_cli.session_path(project_id, session_id)
    self.get_logger().debug("DF_CLIENT: Session Path: %s" % format(self._session))

    self._responses = []
    self._results_pub = self.create_publisher(DialogflowResult, results_topic, 1)
    text_req_topic = requests_topic + '/string_msg'
    
    self._text_req_sub = self.create_subscription(String, text_req_topic, self._text_request_cb, 10)
    self._start_srv = self.create_service(Empty, '/dialogflow_client/start', self._start_dialog_cb)
    self._stop_srv = self.create_service(Empty, '/dialogflow_client/stop', self._stop_dialog_cb)

    """ Audio setup """
    # Mic stream input setup
    self.audio = pyaudio.PyAudio()

    if self.PLAY_AUDIO:
      self._create_audio_output()

    self.get_logger().debug("DF_CLIENT: Last Contexts: %s" % format(self.last_contexts))
    self.get_logger().info("DF_CLIENT: Ready!")

  # ========================================= #
  #           ROS Utility Functions           #
  # ========================================= #

  def _text_request_cb(self, msg):
    """ROS Callback that sends text received from a topic to Dialogflow.

    :param msg: A String message.
    :type msg: String
    """
    self.get_logger().debug("DF_CLIENT: Request received")
    new_msg = DialogflowRequest(query_text = msg.data)
    df_msg = self._detect_intent_text(new_msg)
  
  def _start_dialog_cb(self, req, res):
    self.get_logger().debug("DF_CLIENT: Start service...")
    self._detect_intent_stream()
    return res

  def _stop_dialog_cb(self, req, res):
    Self.get_logger().debug("DF_CLIENT: Stop service...")
    self._responses.cancel()
    return res

  # ----------------- #
  #  Audio Utilities  #
  # ----------------- #

  def _create_audio_output(self):
    """Create a PyAudio output stream."""
    self.get_logger().debug("DF_CLIENT: Creating audio output...")
    self.stream_out = self.audio.open(
      format=pyaudio.paInt16,
      channels=1,
      rate=24000,
      output=True)

  def _play_stream(self, data):
    """Plays the output Dialogflow response.

    :param data: Audio in bytes.
    """
    self.stream_out.start_stream()
    self.stream_out.write(data)
    time.sleep(0.2)  # Wait for stream to finish
    self.stream_out.stop_stream()

  # -------------- #
  #  DF Utilities  #
  # -------------- #

  def _generator(self):
    """Generates yields audio chunks from the buffer.
    Used to stream data to the Google Speech API Asynchronously.

    :return A streaming request with the audio data.
    First request carries config data per Dialogflow docs.
    :rtype: Iterator[:class:`StreamingDetectIntentRequest`]
    """
    # First message contains session, query_input, and params
    query_input = QueryInput(audio_config=self._audio_config)
    contexts = utils.converters.contexts_msg_to_struct(self.last_contexts)
    params = QueryParameters(contexts=contexts)
    req = StreamingDetectIntentRequest(
      session=self._session,
      query_input=query_input,
      query_params=params,
      single_utterance=True,
      output_audio_config=self._output_audio_config
    )
    yield req

    with MicrophoneStream() as stream:
      audio_generator = stream.generator()
      for content in audio_generator:
        yield StreamingDetectIntentRequest(input_audio=content)

  # ======================================== #
  #           Dialogflow Functions           #
  # ======================================== #

  def _detect_intent_text(self, msg):
    """Use the Dialogflow API to detect a user's intent. Goto the Dialogflow
    console to define intents and params.

    :param msg: DialogflowRequest msg
    :return query_result: Dialogflow's query_result with action parameters
    :rtype: DialogflowResult
    """
    # Create the Query Input
    text_input = TextInput(text=msg.query_text, language_code=self._language_code)
    query_input = QueryInput(text=text_input)
    
    try:
      response = self._session_cli.detect_intent(
        session=self._session,
        query_input=query_input
      )
    except google.api_core.exceptions.ServiceUnavailable:
      self.get_logger().warn("DF_CLIENT - detect_intent_text: Deadline exceeded exception caught. The response "
                    "took too long or you aren't connected to the internet!")
    else:
      # Store context for future use
      self.last_contexts = utils.converters.contexts_struct_to_msg(
        response.query_result.output_contexts
      )
      df_msg = utils.converters.result_struct_to_msg(
        response.query_result)
      self.get_logger().warn("DF_CLIENT: Response from DF received. Publishing...")
      self._results_pub.publish(df_msg)
      #self.get_logger().debug(utils.output.print_result(response.query_result))
      # Play audio
      if self.PLAY_AUDIO:
        self._play_stream(response.output_audio)
      return df_msg

  def _detect_intent_stream(self, return_result=False):
    """Gets data from an audio generator (mic) and streams it to Dialogflow.
    We use a stream for VAD and single utterance detection."""
    # Generator yields audio chunks.
    requests = self._generator()
    try:
      self._responses = self._session_cli.streaming_detect_intent(requests)
      resp_list = []
      for response in self._responses:
        resp_list.append(response)
        self.get_logger().debug(
          'DF_CLIENT: Intermediate transcript: "%s".' % 
          format(response.recognition_result.transcript.encode('utf-8'))
        )
        self.get_logger().warn("DF_CLIENT: Reading from mic...")
    except google.api_core.exceptions.Cancelled as c:
      self.get_logger().warn("DF_CLIENT - detect_intent_stream: Caught a Google API Client cancelled "
        "exception. Check request format!:\n %s" % format(c))
    except google.api_core.exceptions.Unknown as u:
      self.get_logger().warn("DF_CLIENT - detect_intent_stream: Unknown Exception Caught:\n %s" % format(u))
    except google.api_core.exceptions.ServiceUnavailable:
      self.get_logger().warn("DF_CLIENT - detect_intent_stream: Deadline exceeded exception caught. The response "
        "took too long or you aren't connected to the internet!")
    except Exception as e:
      self.get_logger().warn("DF_CLIENT - detect_intent_stream: Unexpected exception: %s" % format(e))
    else:
      if response is None:
          self.get_logger().warn("DF_CLIENT - detect_intent_stream: No response received!")
          return None
      # The response list returns responses in the following order:
      # 1. All intermediate recognition results
      # 2. The Final query recognition result (no audio!)
      # 3. The output audio with config
      final_result = resp_list[-2].query_result
      final_audio = resp_list[-1]
      self.last_contexts = utils.converters.contexts_struct_to_msg(
              final_result.output_contexts
      )
      df_msg = utils.converters.result_struct_to_msg(final_result)
      # Pub
      self._results_pub.publish(df_msg)
      self.get_logger().info(utils.output.print_result(final_result))
      # Play audio
      if self.PLAY_AUDIO:
        self._play_stream(final_audio.output_audio)
      if return_result: return df_msg, final_result
      self._responses = []
      return df_msg

  def start(self):
    """Start the dialogflow client"""
    self.get_logger().info("DF_CLIENT: Spinning...")
  
def main(args=None):
  rclpy.init(args=args)
  node = DialogflowClient()
  try:
    node.start()
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass

  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()