from bag_loader import BagLoader
from sqlite_query import SQLiteQuery


def main():
    db_path = "./data/rosbag2_2025_08_12-17_26_19.db"
    bag_path = "./data/rosbag2_2025_08_12-17_26_19"  # mcap 形式の bag ディレクトリ

    # rosbag → sqlite に変換
    loader = BagLoader(bag_path, db_path)
    loader.convert()

    # SQLite 検索
    query = SQLiteQuery(db_path)
    print("Time range:", query.search_by_time_range(0, 999999999999))
    print("Time range None:", query.search_by_time_range(None, None))

    print("Node filter:", query.search_by_node("my_node"))
    print("Level=20:", query.search_by_level(20))
    print("Level>=30:", query.search_by_min_level(30))
    print("Message contains 'ERROR':", query.search_by_message("ERROR"))


if __name__ == "__main__":
    main()
