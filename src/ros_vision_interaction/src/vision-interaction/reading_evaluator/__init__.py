#!/usr/bin/env python3.8
import collections
import contextlib
import logging
import vision_project_tools.reading_task_tools as reading_task_tools
import wave
import webrtcvad

from pydub import AudioSegment
from vision_project_tools.constants import DatabaseKeys
from vision_project_tools.reading_task_tools import TaskDataKeys

logging.basicConfig(level=logging.INFO)


class Frame(object):
    """Represents a "frame" of audio data."""

    def __init__(self, bytes, timestamp, duration):
        self.bytes = bytes
        self.timestamp = timestamp
        self.duration = duration


class ReadingEvaluator:
    """Computes total speaking time in a given audio file."""

    def __init__(
            self,
            statedb
    ):
        self._state_database = statedb
        self._vad = webrtcvad.Vad()
        self._vad.set_mode(3)

    def calculate_and_set_reading_score(self, audio_file_path):
        logging.info(f"Calculating reading score from: {audio_file_path}")
        total_speaking_time = self.get_total_speaking_time(audio_file_path)
        task_id = self._state_database.get(DatabaseKeys.CURRENT_READING_ID)

        if self._state_database.get(DatabaseKeys.UNABLE_TO_READ) == "Unable to read":
            logging.info("Unable to read")
            reading_task_tools.set_reading_task_value(self._state_database, task_id, TaskDataKeys.UNABLE_TO_READ, True)
            self._state_database.set(DatabaseKeys.UNABLE_TO_READ, False)
        else:
            if task_id[0] == "3":  # SPOT READING
                reading_speed = total_speaking_time
            else:
                num_of_words = reading_task_tools.get_reading_task_data_value(
                    self._state_database,
                    task_id,
                    TaskDataKeys.WORD_COUNT
                )
                if total_speaking_time == 0:
                    logging.info("Total speaking time was 0, setting reading speed to 0")
                    logging.info("Total speaking time was 0, setting reading speed to 0")
                    reading_speed = 0
                else:
                    reading_speed = num_of_words / total_speaking_time
                    logging.info(f"Reading speed: {reading_speed}")
                    logging.info(f"Reading speed: {reading_speed}")
            reading_task_tools.set_reading_task_value(self._state_database, task_id, TaskDataKeys.SCORE, reading_speed)
            print(f"----- Reading score: {reading_speed} -----")
        self._set_reading_scores()

    def get_total_speaking_time(self, audio_file_path):
        """Reads a .wav file and returns the total length of speaking time (float)."""
        total_speaking_time = 0.0
        audio, sample_rate = self.read_wav(audio_file_path)
        if not audio:
            logging.info("No audio data, returning 0 for total speaking time")
        else:
            frame_duration_ms = 30
            padding_duration_ms = 300
            frames = list(self.generate_frames(frame_duration_ms, audio, sample_rate))
            voiced_segments = self.get_voiced_audio_from_frames(
                sample_rate,
                frame_duration_ms,
                padding_duration_ms,
                frames
            )
            for segment in voiced_segments:
                audio_segment = AudioSegment(segment, sample_width=2, frame_rate=sample_rate, channels=1)
                total_speaking_time += audio_segment.duration_seconds
                logging.info(f"Adding speaking time: {audio_segment.duration_seconds}")
        return total_speaking_time

    def read_wav(self, path):
        """Reads a .wav file.
        Takes the path, and returns (PCM audio data, sample rate).
        """
        try:
            with contextlib.closing(wave.open(path, 'rb')) as wf:
                num_channels = wf.getnchannels()
                if num_channels != 1:
                    raise ValueError("Invalid number of channels")
                sample_width = wf.getsampwidth()
                if sample_width != 2:
                    raise ValueError("Invalid sample width")
                sample_rate = wf.getframerate()
                if sample_rate not in (8000, 16000, 32000, 48000):
                    raise ValueError("Invalid sample rate")
                pcm_data = wf.readframes(wf.getnframes())
        except Exception as e:
            logging.info(e)
            logging.info(f"Could not read wav file: {path}. Setting PCM data to None and sample rate to 16000.")
            pcm_data = None
            sample_rate = 16000
        return pcm_data, sample_rate

    def generate_frames(self, frame_duration_ms, audio, sample_rate):
        """Generates audio frames from PCM audio data.
        Takes the desired frame duration in milliseconds, the PCM data, and
        the sample rate.
        Yields Frames of the requested duration.
        """
        n = int(sample_rate * (frame_duration_ms / 1000.0) * 2)
        offset = 0
        timestamp = 0.0
        duration = (float(n) / sample_rate) / 2.0
        while offset + n < len(audio):
            yield Frame(audio[offset:offset + n], timestamp, duration)
            timestamp += duration
            offset += n

    def get_voiced_audio_from_frames(
            self,
            sample_rate,
            frame_duration_ms,
            padding_duration_ms,
            frames
    ):
        """Filters out non-voices audio frames and yields the resulting voices audio data in bytes."""
        num_padding_frames = int(padding_duration_ms / frame_duration_ms)
        ring_buffer = collections.deque(maxlen=num_padding_frames)
        triggered = False

        voiced_frames = []
        for frame in frames:
            is_speech = self._vad.is_speech(frame.bytes, sample_rate)

            if not triggered:
                ring_buffer.append((frame, is_speech))
                num_voiced = len([f for f, speech in ring_buffer if speech])
                if num_voiced > 0.9 * ring_buffer.maxlen:
                    triggered = True
                    for f, s in ring_buffer:
                        voiced_frames.append(f)
                    ring_buffer.clear()
            else:
                voiced_frames.append(frame)
                ring_buffer.append((frame, is_speech))
                num_unvoiced = len([f for f, speech in ring_buffer if not speech])
                if num_unvoiced > 0.9 * ring_buffer.maxlen:
                    triggered = False
                    yield b''.join([f.bytes for f in voiced_frames])
                    ring_buffer.clear()
                    voiced_frames = []
        # Yield any leftover frames
        if voiced_frames:
            yield b''.join([f.bytes for f in voiced_frames])

    def _set_reading_scores(self):
        task_id = self._state_database.get(DatabaseKeys.CURRENT_READING_ID)
        score = reading_task_tools.get_reading_task_data_value(self._state_database, task_id, TaskDataKeys.SCORE)

        self._state_database.set(DatabaseKeys.LAST_SCORES, score)

        all_scores = reading_task_tools.get_all_scores(self._state_database)
        try:
            len(all_scores)
        except TypeError:
            all_scores = []
        if len(all_scores) == 0 or score > max(all_scores):
            self._state_database.set(DatabaseKeys.BEST_SCORES, score)

