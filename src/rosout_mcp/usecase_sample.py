from bag_loader import BagLoader
from sqlite_query import SQLiteQuery


def main():
    db_path = "./data/rosout_test.db"
    bag_path = "./data/rosout_test"  # mcap

    print("=== Duplicate Data Problem Solutions ===")
    print("You can choose from the following options:")
    print("1. clear_existing=True (default): Delete existing data before conversion each time")
    print("2. clear_existing=False: Add data (traditional behavior, with duplicates)")
    print()

    # Option 1: Clear DB before conversion each time (recommended)
    print(">>> Using Option 1: Clear existing data before conversion")
    loader = BagLoader(bag_path, db_path)
    loader.convert(clear_existing=True)  # Default, so can be omitted

    # Option 2 example (commented out):
    # print(">>> Using Option 2: Add data (possible duplicates)")
    # loader = BagLoader(bag_path, db_path)
    # loader.convert(clear_existing=False)

    print()

    # SQLite search
    query = SQLiteQuery(db_path)
    # print("Time range:", query.search_by_time_range(0, 1754896147385480395))
    # print("Time range None:", query.search_by_time_range(None, None))
    # print("Node filter:", query.search_by_node("rosbag2_recorder"))
    # print("Level>=30:", query.search_by_level_range(30, None))
    # print("Level<=20:", query.search_by_level_range(None, 20))
    # print("Level>=20 and <=30:", query.search_by_level_range(20, 30))
    # print("Message contains 'Subscribed':",
    #       query.search_by_message("Subscribed"))

    print("\n=== Use case examples for search method ===")

    # 1. Composite search specifying all conditions
    print("\n1. All conditions specified (time range + node + level range + message keyword):")
    results = query.search(
        start_time=0,
        end_time=1754896147385480395,
        node="rosbag2_recorder",
        min_level=10,
        max_level=40,
        message="Subscribed"
    )
    print(f"   Result count: {len(results)}")
    if results:
        print(f"   First result: {results[0]}")

    # 2. Combination of time range and node
    print("\n2. Time range + Node:")
    results = query.search(
        start_time=0,
        end_time=1754896147385480395,
        node="rosbag2_recorder"
    )
    print(f"   Result count: {len(results)}")

    # 3. Combination of log level and message keyword
    print("\n3. Log level range + Message keyword:")
    results = query.search(
        min_level=20,
        max_level=40,
        message="topic"
    )
    print(f"   Result count: {len(results)}")

    # 4. Filter error level or above logs by time range
    print("\n4. Error level or above + Time range:")
    results = query.search(
        start_time=0,
        end_time=1754896147385480395,
        min_level=40  # ERROR or above
    )
    print(f"   Result count: {len(results)}")

    # 5. Logs at warning level or below from specific node
    print("\n5. Specific node + Warning level or below:")
    results = query.search(
        node="rosbag2_recorder",
        max_level=30  # WARN or below
    )
    print(f"   Result count: {len(results)}")

    # 6. Message keyword only
    print("\n6. Message keyword only:")
    results = query.search(
        message="Subscribed"
    )
    print(f"   Result count: {len(results)}")

    # 7. No conditions (get all records)
    print("\n7. No conditions (get all records):")
    results = query.search()
    print(f"   Result count: {len(results)}")


if __name__ == "__main__":
    main()
