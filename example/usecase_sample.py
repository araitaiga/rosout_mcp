# Add src directory to PYTHONPATH
import os
import sys

from src.rosout_mcp.bag_loader import BagLoader
from src.rosout_mcp.db_manager import FileDatabaseManager
from src.rosout_mcp.db_manager import InMemoryDatabaseManager
from src.rosout_mcp.sqlite_query import SQLiteQuery


def demo_file_based_workflow():
    """Demonstrate file-based database workflow with dependency injection."""
    print("="*60)
    print("=== File-Based Database Workflow ===")
    print("="*60)

    bag_path = "./data/rosbag2_2025_08_12-17_26_19"
    db_path = "./data/rosout_demo.db"

    # Create database manager
    db_manager = FileDatabaseManager(db_path)

    try:
        print(f">>> Using file database: {db_path}")
        print(">>> Database manager created successfully")

        # Create components with injected database manager
        loader = BagLoader(bag_path, db_manager)
        query = SQLiteQuery(db_manager)

        print(">>> BagLoader and SQLiteQuery created with shared database manager")

        # Convert rosbag data
        print("\n=== Converting RosBag Data ===")
        loader.convert(clear_existing=True)

        # Perform searches
        perform_search_examples(query)

    finally:
        # Clean up
        db_manager.close()
        print(f"\n>>> Database closed: {db_path}")


def demo_in_memory_workflow():
    """Demonstrate in-memory database workflow with dependency injection."""
    print("\n" + "="*60)
    print("=== In-Memory Database Workflow ===")
    print("="*60)

    bag_path = "./data/rosbag2_2025_08_12-17_26_19"

    # Create in-memory database manager
    db_manager = InMemoryDatabaseManager()

    try:
        print(">>> Using in-memory database")
        print(">>> Database manager created successfully")

        # Create components with injected database manager
        loader = BagLoader(bag_path, db_manager)
        query = SQLiteQuery(db_manager)

        print(">>> BagLoader and SQLiteQuery created with shared in-memory database")

        # Convert rosbag data
        print("\n=== Converting RosBag Data to Memory ===")
        loader.convert(clear_existing=True)

        # Perform searches
        perform_search_examples(query)

    finally:
        # Clean up
        db_manager.close()
        print("\n>>> In-memory database closed")


def perform_search_examples(query):
    """Perform various search examples."""
    print("\n=== Search Examples ===")

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
    print(f"   Total logs: {len(results)}")

    # 8. Error logs only
    print("\n8. Error logs only:")
    error_results = query.search(min_level=40)
    print(f"   Error logs: {len(error_results)}")

    # 9. Info logs only
    print("\n9. Info logs only:")
    info_results = query.search(min_level=20, max_level=20)
    print(f"   Info logs: {len(info_results)}")


def main():
    """Main demonstration function."""
    print("🚀 ROS Bag Log Analysis with Dependency Injection")
    print("This demo shows both file-based and in-memory database approaches")
    print("using the new dependency injection design.\n")

    # Demo 1: File-based workflow
    demo_file_based_workflow()

    # Demo 2: In-memory workflow
    demo_in_memory_workflow()

    print("\n" + "="*60)
    print("✅ Demo completed successfully!")
    print("Both file-based and in-memory approaches work seamlessly")
    print("with the new dependency injection design.")
    print("="*60)


if __name__ == "__main__":
    main()
