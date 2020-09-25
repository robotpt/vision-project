#!/usr/bin/env python3.8

from email_keyword_matcher import EmailKeywordMatcher
import os
import rospy
import typing

from std_msgs.msg import String


class VideoFeedbackEmailer:

    def __init__(self):
        # get environment variables
        to_email = os.environ.get('EMAIL_TO')
        from_email = os.environ.get('ROBOT_EMAIL_ADDRESS')
        email_password = os.environ.get('ROBOT_EMAIL_PASSWORD')
        email_host = os.environ.get('ROBOT_EMAIL_HOST')
        email_port = os.environ.get('ROBOT_EMAIL_PORT')
        # Make sure all environment variables a value
        if not all([to_email, from_email, email_password, email_host, email_port]):
            raise ValueError("Not all OS variables are populated")

        # Setup the email keyword matcher
        keywords = rospy.get_param('video_feedback_emailer/feedback_keywords')
        if type(keywords) is not list:
            raise ValueError('Keywords must be a list')
        self._email_keyword_matcher = self._init_email_keyword_matcher(
            from_email, email_password, email_host, int(email_port),
            keywords)

        self._to_email = to_email
        self._from_email = from_email
        self._subject = rospy.get_param('video_feedback_emailer/email_subject')
        self._content = rospy.get_param('video_feedback_emailer/email_content')
        self._sleep_duration = rospy.get_param('video_feedback_emailer/sleep_seconds')

        # Setup ROS
        rospy.init_node('vision_feedback_emailer')
        rospy.Subscriber(
            rospy.get_param('uploader/publish_topic'),
            String,
            self._data_sent_callback,
            queue_size=1,
        )
        self._feedback_publisher = rospy.Publisher(
            rospy.get_param('video_feedback_emailer/feedback_topic'),
            String,
            queue_size=1
        )

    def _init_email_keyword_matcher(
            self,
            from_email: str,
            email_password: str,
            email_host: str,
            email_port: int,
            keywords: typing.List[str]) -> EmailKeywordMatcher:
        email_keyword_matcher = EmailKeywordMatcher(
            email_address=from_email,
            password=email_password,
            host=email_host,
            port=email_port,
        )
        for keyword in keywords:
            def publish_keyword():
                msg = String()
                msg.data = keyword
                rospy.loginfo(f"Message recieved with keyword '{keyword}'")
                self._feedback_publisher.publish(keyword)

            email_keyword_matcher.add_keyword(keyword, publish_keyword)

        return email_keyword_matcher

    def _data_sent_callback(self, _):
        self._email_keyword_matcher.send(
            to_email=self._to_email,
            subject=self._subject, contents=self._content)

    def run(self):
        while not rospy.is_shutdown():
            while not self._email_keyword_matcher.is_response(
                    from_email=self._from_email, subject=self._subject):
                rospy.sleep(self._sleep_duration)
            self._email_keyword_matcher.process_received(
                from_email=self._from_email, subject=self._subject)


if __name__ == '__main__':
    try:
        VideoFeedbackEmailer().run()
    except rospy.ROSInterruptException:
        pass
