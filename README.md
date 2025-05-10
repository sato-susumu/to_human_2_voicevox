# to_human_2_voicevox

## 導入方法、ビルド
```
cd ~/your_ws/src
git clone https://github.com/sato-susumu/to_human_2_voicevox.git
cd ~/your_ws
colcon build --packages-select to_human_2_voicevox
source install/setup.bash
```

## デフォルト起動（/to_humanを購読）
```
ros2 run to_human_2_voicevox to_human_2_voicevox
```

## 起動時にトピックを変更
```
ros2 run to_human_2_voicevox to_human_2_voicevox \
  --ros-args -p input_topic:="/from_human" -p character_id:=5
```

## 動作確認用トピック発行
```
ros2 topic pub --once /to_human std_msgs/msg/String "{data: 'こんにちは！テストです。'}"
```


## 動的パラメータ変更（キャラIDのみ）
```
ros2 param set /to_human_2_voicevox character_id 7
```

