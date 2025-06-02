import time
import re
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String as StdString
from voicevox_ros2_interface.srv import Speaker as SpeakerService

# 定数定義
DEFAULT_NODE_NAME = 'to_human_2_voicevox'
DEFAULT_CHARACTER_ID = 2
DEFAULT_INPUT_TOPIC = '/to_human'
DEFAULT_OUTPUT_SERVICE_NAME = '/voicevox_ros2/speaker_srv'
DELIMITERS = '、。！？'

class ToHuman2VoicevoxNode(Node):
    def __init__(self) -> None:
        super().__init__(DEFAULT_NODE_NAME)
        # パラメータとデフォルト値
        self.declare_parameter('character_id', DEFAULT_CHARACTER_ID)
        self.declare_parameter('input_topic', DEFAULT_INPUT_TOPIC)
        self.character_id: int = self.get_parameter('character_id').value
        topic: str = self.get_parameter('input_topic').value

        # キャラID変更コールバック登録
        self.add_on_set_parameters_callback(self._on_parameter_event)

        # Service client: 定数で指定されたサービス名
        self.client = self.create_client(SpeakerService, DEFAULT_OUTPUT_SERVICE_NAME)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(f'{DEFAULT_OUTPUT_SERVICE_NAME} is not available, waiting...')

        # Subscriber: 指定トピック（Reliability を合わせる）
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(
            StdString,
            topic,
            self._on_to_human,
            qos_profile=qos,
        )

        self.get_logger().info(f"Node '{DEFAULT_NODE_NAME}' started. Listening on '{topic}'")
        
        # 処理中のフラグ
        self._is_processing = False
        self._text_queue = []

    def _on_parameter_event(self, params: List[Parameter]) -> SetParametersResult:
        for p in params:
            if p.name == 'character_id' and p.type_ == Parameter.Type.INTEGER:
                self.get_logger().info(f'character_id → {p.value}')
                self.character_id = p.value
        return SetParametersResult(successful=True)

    def _split_text(self, text: str) -> List[str]:
        """
        入力テキストを区切り文字（、。！？）で分割し、区切り文字を末尾に含めた形で返します。
        連続する区切り文字（例：「！？」）は1つのグループとして扱われます。

        Args:
            text (str): 分割対象のテキスト

        Returns:
            List[str]: 区切り文字を含む分割されたテキストのリスト

        例：
            入力: "こんにちは、元気ですか？今日はいい天気ですね！"
            出力: ["こんにちは、", "元気ですか？", "今日はいい天気ですね！"]
        """
        # 区切り文字のパターンを作成
        # 例: ([^、。！？]+[、。！？]+|[^、。！？]+$) という正規表現になり、
        # 「区切り文字以外の文字列 + 区切り文字」または「区切り文字以外の文字列で終わる」パターンにマッチします
        pattern = f'([^{DELIMITERS}]+[{DELIMITERS}]+|[^{DELIMITERS}]+$)'
        
        # 正規表現でマッチした部分を抽出
        parts = re.findall(pattern, text)
        return parts

    def _on_to_human(self, msg: StdString) -> None:
        self.get_logger().info(f"Received on '{DEFAULT_INPUT_TOPIC}': {msg.data}")
        
        # テキストを分割
        text_parts = self._split_text(msg.data)
        self._text_queue.extend(text_parts)
        
        # まだ処理中でなければ、処理を開始
        if not self._is_processing:
            self._process_next_text()

    def _process_next_text(self) -> None:
        if not self._text_queue:
            self._is_processing = False
            return

        self._is_processing = True
        text = self._text_queue.pop(0)

        # 入力文字列をサービスリクエストとして送信
        req = SpeakerService.Request()
        req.text = text
        req.id = self.character_id

        # リクエスト内容をログに出力
        self.get_logger().info(f'Sending request to SpeakerService: text="{text}", character_id={self.character_id}')

        # 時間計測開始
        start_time = time.time()
        future = self.client.call_async(req)
        future.add_done_callback(lambda fut: self._on_speak_response(fut, start_time))

    def _on_speak_response(self, future, start_time: float) -> None:
        # 時間計測終了
        duration = time.time() - start_time
        res = future.result()
        if res is not None and res.success:
            self.get_logger().info(f'speak succeeded (took {duration:.3f} sec)')
        else:
            self.get_logger().error(f'speak failed (took {duration:.3f} sec)')
        
        # 次のテキストを処理
        self._process_next_text()

def main() -> None:
    rclpy.init()
    node = ToHuman2VoicevoxNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
