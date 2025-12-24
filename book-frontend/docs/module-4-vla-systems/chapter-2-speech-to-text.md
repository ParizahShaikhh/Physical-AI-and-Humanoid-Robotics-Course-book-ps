# Chapter 2: Speech-to-Text with OpenAI Whisper

## Learning Objectives

By the end of this chapter, you will be able to:
- Integrate OpenAI Whisper for speech-to-text conversion in robotics applications
- Configure Whisper models for real-time robotic command processing
- Handle various audio conditions and optimize transcription accuracy
- Process audio streams in ROS 2 nodes for robotic applications

## Introduction to Speech-to-Text in Robotics

Speech-to-text processing serves as the critical interface between human natural language commands and robotic systems. In the Vision-Language-Action paradigm, the speech-to-text component converts spoken commands into text format that can be processed by the language understanding system.

OpenAI Whisper has emerged as a leading model for automatic speech recognition (ASR), offering robust performance across multiple languages and audio conditions. For robotic applications, Whisper provides the accuracy and reliability needed to enable natural human-robot interaction.

## OpenAI Whisper Architecture

### Model Overview

Whisper is a transformer-based model that processes audio spectrograms to generate text transcriptions. The model architecture includes:

1. **Audio Encoder**: Processes audio inputs and extracts features
2. **Text Decoder**: Generates text tokens based on audio features
3. **Multilingual Capability**: Supports multiple languages in a single model

### Model Variants

Whisper offers several model sizes optimized for different applications:

- **tiny**: Fastest processing, suitable for real-time applications
- **base**: Good balance of speed and accuracy
- **small**: Higher accuracy with moderate processing requirements
- **medium**: High accuracy for most robotic applications
- **large**: Highest accuracy for precision applications

```python
import whisper

# Load different model variants
model_tiny = whisper.load_model("tiny")
model_base = whisper.load_model("base")
model_medium = whisper.load_model("medium")
```

## Whisper Integration in Robotic Systems

### Audio Preprocessing

Robotic applications require careful audio preprocessing to ensure optimal Whisper performance:

```python
import numpy as np
import librosa
from scipy import signal

def preprocess_audio(audio_data, sample_rate=16000):
    """
    Preprocess audio for Whisper processing
    """
    # Resample to Whisper's expected sample rate
    if sample_rate != 16000:
        audio_data = librosa.resample(audio_data, orig_sr=sample_rate, target_sr=16000)

    # Normalize audio levels
    audio_data = audio_data / np.max(np.abs(audio_data))

    # Apply noise reduction if needed
    # (Implementation depends on specific requirements)

    return audio_data
```

### Real-Time Processing Pipeline

For robotic applications, Whisper can be integrated into a real-time processing pipeline:

```python
import whisper
import rospy
import threading
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String

class WhisperSTTNode:
    def __init__(self):
        rospy.init_node('whisper_stt_node')

        # Load Whisper model
        self.model = whisper.load_model("base")

        # Audio buffer for continuous processing
        self.audio_buffer = []
        self.buffer_lock = threading.Lock()

        # Publishers and subscribers
        self.audio_sub = rospy.Subscriber('/audio_input', AudioData, self.audio_callback)
        self.text_pub = rospy.Publisher('/transcribed_text', String, queue_size=10)

    def audio_callback(self, audio_msg):
        """
        Process incoming audio data
        """
        with self.buffer_lock:
            # Convert audio message to numpy array
            audio_array = np.frombuffer(audio_msg.data, dtype=np.int16).astype(np.float32)
            audio_array /= 32768.0  # Normalize to [-1, 1]

            self.audio_buffer.extend(audio_array)

            # Process when sufficient audio is available
            if len(self.audio_buffer) > 16000 * 2:  # 2 seconds of audio at 16kHz
                self.process_audio_buffer()

    def process_audio_buffer(self):
        """
        Process accumulated audio buffer with Whisper
        """
        if len(self.audio_buffer) == 0:
            return

        audio_array = np.array(self.audio_buffer)

        # Transcribe audio
        result = self.model.transcribe(audio_array)
        transcription = result['text']

        # Publish transcription
        text_msg = String()
        text_msg.data = transcription
        self.text_pub.publish(text_msg)

        # Clear buffer
        self.audio_buffer = []

    def run(self):
        """
        Run the STT node
        """
        rospy.spin()

if __name__ == '__main__':
    stt_node = WhisperSTTNode()
    stt_node.run()
```

## Audio Processing Considerations

### Sample Rate and Format

Whisper expects audio at 16kHz sample rate with floating-point values in the range [-1, 1]. Robotic systems must ensure proper audio preprocessing:

```python
def ensure_proper_format(audio_data, sample_rate):
    """
    Ensure audio is in Whisper-compatible format
    """
    # Resample to 16kHz
    if sample_rate != 16000:
        audio_data = librosa.resample(audio_data, orig_sr=sample_rate, target_sr=16000)

    # Convert to float32 if needed
    if audio_data.dtype != np.float32:
        audio_data = audio_data.astype(np.float32)

    # Normalize to [-1, 1] range
    if audio_data.max() > 1.0 or audio_data.min() < -1.0:
        audio_data = audio_data / np.max(np.abs(audio_data))

    return audio_data
```

### Noise Handling

Robotic environments often have background noise that can affect transcription accuracy:

```python
import webrtcvad
from scipy import signal

class NoiseRobustSTT:
    def __init__(self):
        # Initialize VAD (Voice Activity Detection)
        self.vad = webrtcvad.Vad(2)  # Aggressiveness level 2
        self.sample_rate = 16000

    def detect_voice_activity(self, audio_chunk):
        """
        Detect voice activity in audio chunk
        """
        # Convert to 16-bit for VAD
        audio_16bit = (audio_chunk * 32767).astype(np.int16)

        # VAD works on 10ms, 20ms, or 30ms frames
        frame_size = int(0.01 * self.sample_rate)  # 10ms frame
        frames = self.frame_generator(audio_16bit, frame_size)

        voice_detected = False
        for frame in frames:
            if self.vad.is_speech(frame.tobytes(), self.sample_rate):
                voice_detected = True
                break

        return voice_detected

    def frame_generator(self, audio, frame_size):
        """
        Generate audio frames for VAD processing
        """
        for i in range(0, len(audio), frame_size):
            yield audio[i:i+frame_size]
```

## Whisper Configuration for Robotics

### Model Selection Strategy

For robotic applications, model selection depends on computational resources and real-time requirements:

```python
class WhisperConfig:
    @staticmethod
    def select_model_by_requirements(latency_sensitivity, accuracy_requirement, compute_power):
        """
        Select appropriate Whisper model based on requirements
        """
        if latency_sensitivity == "high" and compute_power == "limited":
            return "tiny"
        elif accuracy_requirement == "high" and compute_power == "abundant":
            return "large"
        elif latency_sensitivity == "medium" and accuracy_requirement == "medium":
            return "base"
        else:
            return "medium"  # Default choice

    @staticmethod
    def get_processing_parameters(model_size):
        """
        Get processing parameters based on model size
        """
        params = {
            "tiny": {"batch_size": 1, "max_tokens": 448, "processing_delay": 0.5},
            "base": {"batch_size": 1, "max_tokens": 512, "processing_delay": 0.8},
            "small": {"batch_size": 1, "max_tokens": 768, "processing_delay": 1.2},
            "medium": {"batch_size": 1, "max_tokens": 1024, "processing_delay": 2.0},
            "large": {"batch_size": 1, "max_tokens": 1280, "processing_delay": 3.0}
        }
        return params.get(model_size, params["base"])
```

### Language and Task Configuration

Whisper supports multiple languages and can be configured for specific tasks:

```python
def transcribe_with_config(model, audio, language=None, task="transcribe", temperature=0.0):
    """
    Transcribe audio with specific configuration
    """
    options = {
        "language": language,
        "task": task,  # "transcribe" or "translate"
        "temperature": temperature,
        "best_of": 5,
        "beam_size": 5,
        "patience": 1.0
    }

    result = model.transcribe(audio, **options)
    return result
```

## Performance Optimization

### Batch Processing

For improved throughput, audio can be processed in batches:

```python
class BatchWhisperProcessor:
    def __init__(self, model_size="base", batch_size=4):
        self.model = whisper.load_model(model_size)
        self.batch_size = batch_size
        self.audio_queue = []

    def add_audio(self, audio_data):
        """
        Add audio to processing queue
        """
        self.audio_queue.append(audio_data)

        if len(self.audio_queue) >= self.batch_size:
            self.process_batch()

    def process_batch(self):
        """
        Process batch of audio data
        """
        if not self.audio_queue:
            return

        # Process all audio in the queue
        results = []
        for audio in self.audio_queue:
            result = self.model.transcribe(audio)
            results.append(result)

        # Clear queue and return results
        self.audio_queue = []
        return results
```

### Caching and Warm-up

Pre-warm the model to reduce latency for the first transcription:

```python
def warm_up_model(model):
    """
    Warm up Whisper model to reduce first transcription latency
    """
    # Create dummy audio (silent)
    dummy_audio = np.zeros(16000 * 2, dtype=np.float32)  # 2 seconds of silence

    # Process dummy audio to initialize model
    result = model.transcribe(dummy_audio)

    return result
```

## Integration with ROS 2

### Message Definitions

Define custom messages for speech-to-text communication:

```xml
# In vla_speech_msgs/msg/SpeechTranscription.msg
string text
float32 confidence
int32[] tokens
builtin_interfaces/Time timestamp
string language
```

### Launch File

Create a launch file for the speech-to-text system:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vla_speech',
            executable='whisper_stt_node',
            name='whisper_stt',
            parameters=[
                {'model_size': 'base'},
                {'sample_rate': 16000},
                {'language': 'en'},
                {'compute_type': 'float16'}  # Use if GPU available
            ],
            remappings=[
                ('/audio_input', '/microphone/audio_raw'),
                ('/transcribed_text', '/vla/commands/text')
            ]
        )
    ])
```

## Quality and Accuracy Considerations

### Accuracy Under Different Conditions

Whisper performance varies with audio conditions:

```python
def evaluate_audio_quality(audio_data, sample_rate=16000):
    """
    Evaluate audio quality metrics
    """
    # Signal-to-noise ratio estimation
    signal_power = np.mean(audio_data ** 2)
    noise_power = np.var(audio_data - np.mean(audio_data))
    snr = 10 * np.log10(signal_power / (noise_power + 1e-10))

    # Audio level
    rms = np.sqrt(np.mean(audio_data ** 2))

    # Silence detection
    is_silent = rms < 0.01

    return {
        'snr': snr,
        'rms': rms,
        'is_silent': is_silent,
        'estimated_accuracy': min(0.95, max(0.6, 0.8 + (snr - 20) * 0.01))
    }
```

### Confidence Scoring

Use Whisper's internal confidence measures:

```python
def get_transcription_with_confidence(model, audio):
    """
    Get transcription with confidence scoring
    """
    result = model.transcribe(audio, temperature=0.0)

    # Extract confidence from tokens if available
    if 'segments' in result and len(result['segments']) > 0:
        avg_confidence = np.mean([seg.get('avg_logprob', 0) for seg in result['segments']])
        confidence = max(0.0, min(1.0, (avg_confidence + 5) / 5))  # Normalize to [0,1]
    else:
        confidence = 0.5  # Default confidence

    return result['text'], confidence
```

## Troubleshooting and Error Handling

### Common Issues

1. **Audio Format Issues**: Ensure proper sample rate and data type
2. **Memory Constraints**: Use smaller models or reduce batch size
3. **Latency Problems**: Implement streaming or reduce model size
4. **Accuracy Issues**: Preprocess audio or adjust model parameters

### Error Recovery

```python
class RobustWhisperSTT:
    def __init__(self, fallback_model_size="tiny"):
        self.primary_model = whisper.load_model("base")
        self.fallback_model = whisper.load_model(fallback_model_size)

    def robust_transcribe(self, audio):
        """
        Transcribe with fallback mechanism
        """
        try:
            result = self.primary_model.transcribe(audio)
            return result['text'], True  # Success flag
        except Exception as e:
            rospy.logwarn(f"Primary model failed: {e}, falling back to smaller model")
            try:
                result = self.fallback_model.transcribe(audio)
                return result['text'], False  # Fallback used
            except Exception as e2:
                rospy.logerr(f"Both models failed: {e2}")
                return "", False  # Both failed
```

## Summary

OpenAI Whisper provides a robust foundation for speech-to-text processing in robotic systems. By properly configuring the model, preprocessing audio, and integrating with ROS 2, robots can understand natural language commands with high accuracy. The key to success lies in matching model capabilities to computational resources and environmental conditions.

## Exercises

1. Implement a Whisper-based STT node that processes audio from a microphone and publishes transcriptions
2. Compare transcription accuracy between different Whisper model sizes in your environment
3. Add voice activity detection to reduce unnecessary processing of silence
4. Implement confidence-based filtering of transcriptions

## Further Reading

- [OpenAI Whisper GitHub Repository](https://github.com/openai/whisper) - Official implementation and documentation
- [Whisper Paper](https://cdn.openai.com/papers/whisper.pdf) - Technical details and evaluation
- [Real-time Speech Recognition with Whisper](https://arxiv.org/abs/2212.06909) - Streaming approaches