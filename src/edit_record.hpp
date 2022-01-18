#pragma once

#include <functional>

namespace point_categorization
{

	// function types
	typedef std::function<void()> UndoFunc;
	typedef std::function<void()> RedoFunc;
	
	// represents an action that edits the current state of the program.
	// such an action must provide functions to undo and redo its changes
	class EditRecord
	{
		public:
			EditRecord(UndoFunc, RedoFunc);

			void undo() const;
			void redo() const;
		private:
			// internal function objects
			UndoFunc _undo;
			RedoFunc _redo;
	};

}
