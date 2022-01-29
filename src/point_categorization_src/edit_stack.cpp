#include "edit_stack.hpp"

using namespace point_categorization;

bool EditStack::canUndo() const
{
	return (_records.size() > 0);
}

bool EditStack::canRedo() const
{
	return (_undoneRecords.size() > 0);
}

void EditStack::pushEdit(const EditRecord& record)
{
	_records.push_back(record);
	_undoneRecords.clear();
}

void EditStack::undo()
{
	if(!canUndo())
		throw std::runtime_error("Attempt to undo from an empty UndoStack");

	EditRecord record = _records.back();
	_records.pop_back();
	
	record.undo();

	_undoneRecords.push_back(record);
}

void EditStack::redo()
{
	if(!canRedo())
		throw std::runtime_error("Attempt to redo from an UndoStack with no redoable actions.");

	EditRecord record = _undoneRecords.back();
	_undoneRecords.pop_back();

	record.redo();

	_records.push_back(record);	
}

void EditStack::clear()
{
	_records.clear();
	_undoneRecords.clear();
}
