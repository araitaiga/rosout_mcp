"""
Unit tests for bag_loader.py (using mocks)
"""

import pytest
from unittest.mock import MagicMock
from unittest.mock import patch

from rosout_mcp.bag_loader import BagLoader


class TestBagLoader:
    """Test class for BagLoader"""

    def test_init_with_valid_params(self, in_memory_db):
        """Test initialization with valid parameters"""
        loader = BagLoader("/path/to/bag", in_memory_db)
        assert loader.bag_path == "/path/to/bag"
        assert loader.db_manager == in_memory_db

    def test_init_with_none_db_manager(self):
        """Test initialization error with None database manager"""
        with pytest.raises(ValueError, match="db_manager is required"):
            BagLoader("/path/to/bag", None)

    @patch('rosout_mcp.bag_loader.os.path.exists')
    def test_convert_with_nonexistent_path(self, mock_exists, in_memory_db):
        """Test conversion with nonexistent path"""
        mock_exists.return_value = False

        loader = BagLoader("/nonexistent/path", in_memory_db)

        # Should raise FileNotFoundError when path doesn't exist
        with pytest.raises(FileNotFoundError, match="Bag path does not exist: /nonexistent/path"):
            loader.convert()

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
