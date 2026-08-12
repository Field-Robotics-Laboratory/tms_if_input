筑波大永谷研究室が開発を進める情報流通IFとROS2-TMS for Constructionを接続するインタフェース

動作方法：

出力したxmlファイル(ConstructionPlan形式、ver3.xml参照)を/tms_if_input/json_samplesに格納し、以下のコマンドを実行

sudo systemctl start mongod

ros2 launch tms_if_input tms_if_input.launch.py
