#pragma once
#include "edit_record.hpp"

#include <vector>
#include <stdexcept>

namespace point_categorization
{

	// represents a stack of actions which can then be
	// undone / redone
	class EditStack
	{
		public:
			EditStack() = default;
			~EditStack() = default;
		
			void pushEdit(const EditRecord& record);

			bool canUndo();
			bool canRedo();

			void undo();
			void redo();

			void clear();

		private:
			// underlying edit records
			std::vector<EditRecord> _records;

			// records that are removed by undo() are stored
			// here temporarily, in case redo() is called.
			// If redo() is called the record is placed back onto
			// _records. If an action is pushed this vector is cleared.
			std::vector<EditRecord> _undoneRecords;
	};	

}
