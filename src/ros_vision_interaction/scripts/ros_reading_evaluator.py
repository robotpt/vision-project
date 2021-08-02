#!/usr/bin/env python3.8
import glob
import os
import pymongo
import rospy
import sys
import time

from std_msgs.msg import Bool
from vision_project_tools import init_db
from vision_project_tools import EngineStateDb as StateDb
from vision_project_tools.constants import DatabaseKeys, INITIAL_STATE_DB
from reading_evaluator import ReadingEvaluator

sys.path.append('/root/catkin_ws/src/ffmpeg')


class RosReadingEvaluator:

    def __init__(
            self,
            statedb,
            reading_evaluator,
            source_directory,
            file_prefix,
            extension="wav",
            seconds_to_check_for_audio_files=5
    ):
        if not os.path.exists(source_directory):
            raise FileNotFoundError("The upload directory \'{}\' does not exist".format(source_directory))

        self._source_directory = source_directory
        self._file_prefix = file_prefix
        self._extension = extension
        self._seconds_to_check_for_audio_files = seconds_to_check_for_audio_files

        is_record_evaluation_topic = rospy.get_param("vision-project/controllers/is_record/evaluation")
        self._is_record_subscriber = rospy.Subscriber(
            is_record_evaluation_topic,
            Bool,
            callback=self.reading_analyzer_callback,
            queue_size=1
        )

        self._state_database = statedb
        self._reading_evaluator = reading_evaluator

    def reading_analyzer_callback(self, is_record):
        if not is_record.data:
            rospy.loginfo("Evaluator callback starting")
            audio_file = self.find_audio_file(
                self._source_directory,
                self._extension,
                self._file_prefix
            )
            self._reading_evaluator.calculate_and_set_reading_score(audio_file)

    def find_audio_file(
            self,
            directory,
            extension,
            file_prefix
    ):
        """Finds a file based on the directory, location, and file prefix.
        Returns the first occurrence of a valid file path."""
        list_of_audio_files = []
        for _ in range(self._seconds_to_check_for_audio_files):
            rospy.loginfo(f"Looking in directory: {directory}")
            list_of_audio_files = glob.glob(os.path.join(directory, "*." + extension))
            rospy.loginfo(f"Audio files found: {list_of_audio_files}")
            if len(list_of_audio_files) > 0 and \
                    any(file_prefix in string for string in list_of_audio_files):
                break
            time.sleep(0.2)
        for file in list_of_audio_files:
            if file.find(file_prefix) >= 0:
                return file
        rospy.loginfo("No matching files found, returning None")
        return None


if __name__ == "__main__":
    rospy.init_node("reading_evaluator")
    host = rospy.get_param("mongodb/host")
    port = rospy.get_param("mongodb/port")
    database_name = rospy.get_param("mongodb/database_name")
    collection_name = rospy.get_param("mongodb/collection_name")
    state_database = StateDb(
        pymongo.MongoClient(host, port),
        database_name=database_name,
        collection_name=collection_name
    )
    init_db(state_database, INITIAL_STATE_DB)

    source_directory = rospy.get_param("vision-project/data/data_capture")

    reading_evaluator = ReadingEvaluator(state_database)

    ros_evaluator = RosReadingEvaluator(
        statedb=state_database,
        source_directory=source_directory,
        file_prefix="evaluation",
        extension="wav",
        reading_evaluator=reading_evaluator
    )

    rospy.spin()
