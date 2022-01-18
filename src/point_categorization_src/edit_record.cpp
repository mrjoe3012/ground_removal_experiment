#include "edit_record.hpp"

namespace point_categorization
{
	EditRecord::EditRecord(UndoFunc undoFunc, RedoFunc redoFunc)
	:	_undo(undoFunc), _redo(redoFunc)
	{
	}

	void EditRecord::undo() const
	{
		_undo();
	}

	void EditRecord::redo() const
	{
		_redo();
	}
}
