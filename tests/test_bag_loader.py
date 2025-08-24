"""
Unit tests for bag_loader.py (using mocks)
"""
import os

import pytest
from unittest.mock import MagicMock
from unittest.mock import Mock
from unittest.mock import patch

from rosout_mcp.bag_loader import BagLoader


class TestBagLoader:
    """Test class for BagLoader"""

    @pytest.mark.unit
    def test_init_with_valid_params(self, in_memory_db):
        """Test initialization with valid parameters"""
        loader = BagLoader("/path/to/bag", in_memory_db)
        assert loader.bag_path == "/path/to/bag"
        assert loader.db_manager == in_memory_db

    @pytest.mark.unit
    def test_init_with_none_db_manager(self):
        """Test initialization error with None database manager"""
        with pytest.raises(ValueError, match="db_manager is required"):
            BagLoader("/path/to/bag", None)

    @pytest.mark.unit
    def test_clear_db(self, in_memory_db, sample_log_data):
        """Test _clear_db method"""
        from conftest import insert_sample_data

        # Insert data
        insert_sample_data(in_memory_db, sample_log_data)

        # Verify data exists
        result = in_memory_db.execute("SELECT COUNT(*) FROM logs")
        assert result[0][0] == len(sample_log_data)

        # Create BagLoader and execute clear
        loader = BagLoader("/path/to/bag", in_memory_db)
        loader._clear_db()

        # Verify data has been cleared
        result = in_memory_db.execute("SELECT COUNT(*) FROM logs")
        assert result[0][0] == 0

    @pytest.mark.unit
    @patch('rosout_mcp.bag_loader.os.path.exists')
    @patch('rosout_mcp.bag_loader.rosbag2_py.SequentialReader')
    @patch('rosout_mcp.bag_loader.rosbag2_py.StorageOptions')
    @patch('rosout_mcp.bag_loader.rosbag2_py.ConverterOptions')
    @patch('rosout_mcp.bag_loader.get_message')
    @patch('rosout_mcp.bag_loader.deserialize_message')
    def test_convert_with_mock_data(self, mock_deserialize, mock_get_message,
                                    mock_converter_options, mock_storage_options,
                                    mock_reader_class, mock_exists, in_memory_db):
        """Test conversion using mock data"""
        # Setup mocks
        mock_exists.return_value = True
        mock_reader = MagicMock()
        mock_reader_class.return_value = mock_reader

        # Setup mock topic information
        mock_topic_metadata = Mock()
        mock_topic_metadata.name = "/rosout"
        mock_topic_metadata.type = "rcl_interfaces/msg/Log"
        mock_reader.get_all_topics_and_types.return_value = [
            mock_topic_metadata]

        # Setup mock message data
        mock_messages = [
            ("/rosout", b"mock_data_1", 12345),
            ("/rosout", b"mock_data_2", 12346),
            ("/other_topic", b"mock_data_3", 12347),  # Should be ignored
        ]

        call_count = 0

        def mock_has_next():
            nonlocal call_count
            return call_count < len(mock_messages)

        def mock_read_next():
            nonlocal call_count
            result = mock_messages[call_count]
            call_count += 1
            return result

        mock_reader.has_next.side_effect = mock_has_next
        mock_reader.read_next.side_effect = mock_read_next

        # Setup mock message objects
        mock_msg1 = Mock()
        mock_msg1.stamp.sec = 1234567890
        mock_msg1.stamp.nanosec = 123456789
        mock_msg1.name = "test_node_1"
        mock_msg1.level = 20
        mock_msg1.msg = "Test message 1"

        mock_msg2 = Mock()
        mock_msg2.stamp.sec = 1234567891
        mock_msg2.stamp.nanosec = 987654321
        mock_msg2.name = "test_node_2"
        mock_msg2.level = 40
        mock_msg2.msg = "Test message 2"

        mock_deserialize.side_effect = [mock_msg1, mock_msg2]

        # Execute test
        loader = BagLoader("/path/to/bag", in_memory_db)
        loader.convert(clear_existing=True)

        # Verify
        mock_exists.assert_called_once_with("/path/to/bag")
        mock_reader.open.assert_called_once()
        mock_reader.get_all_topics_and_types.assert_called_once()

        # Verify data has been inserted into database
        result = in_memory_db.execute("SELECT * FROM logs ORDER BY timestamp")
        assert len(result) == 2  # 2 entries because /other_topic is ignored

        # Verify first message
        assert result[0][0] == 1234567890000000000 + 123456789  # timestamp
        assert result[0][1] == "test_node_1"  # node
        assert result[0][2] == 20  # level
        assert result[0][3] == "Test message 1"  # message

    @pytest.mark.unit
    @patch('rosout_mcp.bag_loader.os.path.exists')
    def test_convert_with_nonexistent_path(self, mock_exists, in_memory_db):
        """Test conversion with nonexistent path"""
        mock_exists.return_value = False

        loader = BagLoader("/nonexistent/path", in_memory_db)

        # Should raise FileNotFoundError when path doesn't exist
        with pytest.raises(FileNotFoundError, match="Bag path does not exist: /nonexistent/path"):
            loader.convert()

    @pytest.mark.unit
    @patch('rosout_mcp.bag_loader.os.path.exists')
    @patch('rosout_mcp.bag_loader.rosbag2_py.SequentialReader')
    def test_convert_clear_existing_false(self, mock_reader_class, mock_exists, in_memory_db, sample_log_data):
        """Test conversion with clear_existing=False"""
        from conftest import insert_sample_data

        # Insert existing data
        insert_sample_data(in_memory_db, sample_log_data)
        original_count = len(sample_log_data)

        # Setup mocks (no additional data)
        mock_exists.return_value = True
        mock_reader = MagicMock()
        mock_reader_class.return_value = mock_reader
        mock_reader.get_all_topics_and_types.return_value = []
        mock_reader.has_next.return_value = False

        # Execute test
        loader = BagLoader("/path/to/bag", in_memory_db)
        loader.convert(clear_existing=False)

        # Verify existing data is retained
        result = in_memory_db.execute("SELECT COUNT(*) FROM logs")
        assert result[0][0] == original_count

    @pytest.mark.unit
    @patch('rosout_mcp.bag_loader.os.path.exists')
    @patch('rosout_mcp.bag_loader.rosbag2_py.SequentialReader')
    def test_convert_clear_existing_true(self, mock_reader_class, mock_exists, in_memory_db, sample_log_data):
        """Test conversion with clear_existing=True"""
        from conftest import insert_sample_data

        # Insert existing data
        insert_sample_data(in_memory_db, sample_log_data)

        # Setup mocks (no additional data)
        mock_exists.return_value = True
        mock_reader = MagicMock()
        mock_reader_class.return_value = mock_reader
        mock_reader.get_all_topics_and_types.return_value = []
        mock_reader.has_next.return_value = False

        # Execute test
        loader = BagLoader("/path/to/bag", in_memory_db)
        loader.convert(clear_existing=True)

        # Verify data has been cleared
        result = in_memory_db.execute("SELECT COUNT(*) FROM logs")
        assert result[0][0] == 0

    @pytest.mark.unit
    @patch('rosout_mcp.bag_loader.os.path.exists')
    @patch('rosout_mcp.bag_loader.rosbag2_py.SequentialReader')
    def test_convert_no_log_topics(self, mock_reader_class, mock_exists, in_memory_db):
        """Test when no log topics exist"""
        mock_exists.return_value = True
        mock_reader = MagicMock()
        mock_reader_class.return_value = mock_reader

        # Only non-log topics exist
        mock_topic_metadata = Mock()
        mock_topic_metadata.name = "/other_topic"
        mock_topic_metadata.type = "geometry_msgs/msg/Twist"
        mock_reader.get_all_topics_and_types.return_value = [
            mock_topic_metadata]

        # Messages exist but not log topics
        mock_reader.has_next.side_effect = [True, False]
        mock_reader.read_next.return_value = ("/other_topic", b"data", 123)

        # Execute test
        loader = BagLoader("/path/to/bag", in_memory_db)
        loader.convert()

        # Verify no log data has been inserted
        result = in_memory_db.execute("SELECT COUNT(*) FROM logs")
        assert result[0][0] == 0

    @pytest.mark.unit
    def test_convert_with_transaction_error(self, in_memory_db):
        """Test transaction error handling"""
        loader = BagLoader("/path/to/bag", in_memory_db)

        # Mock rosbag2_py import to raise ImportError
        with patch('rosout_mcp.bag_loader.rosbag2_py') as mock_rosbag2:
            mock_rosbag2.StorageOptions.side_effect = ImportError("rosbag2_py not available")

            with patch('rosout_mcp.bag_loader.os.path.exists', return_value=True):
                # Should raise ImportError when rosbag2_py is not available
                with pytest.raises(ImportError, match="rosbag2_py not available"):
                    loader.convert()
