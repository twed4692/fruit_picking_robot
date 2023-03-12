/*--------------------------------------------------------------------------------------------------------
C++ implementation of FruitPicker class. Has a camera subscriber to detect colours, and the following
functionality:

- Counting total fruits picked (the "fruit basket") and current row fruits picked.
- Picking fruits of all colours in the available colours vector, and managing the content of this vector.
- Ignoring fruits that have already been picked on return to the last node reached.

Original revision: 510548722
--------------------------------------------------------------------------------------------------------*/

#include "FruitPicker.h"

/*--------------------------------------------------------------------------------------------------------
Constructor, initialise fruit counting variables to 0.
--------------------------------------------------------------------------------------------------------*/
CFruitPicker::CFruitPicker() 
{
    _numFruits = 0;
    _fruitCountedOnRow = 0;
    _fruitToIgnore = 0;
}


/*--------------------------------------------------------------------------------------------------------
Get the number of fruits currently in the basket.
--------------------------------------------------------------------------------------------------------*/
int CFruitPicker::GetNumFruits()
{
    return _numFruits;
}


/*--------------------------------------------------------------------------------------------------------
Tell the fruit picker we are in a new row, reset count of fruits picked in the current row.
--------------------------------------------------------------------------------------------------------*/
void CFruitPicker::NewRow()
{
    _fruitCountedOnRow = 0;
}


/*--------------------------------------------------------------------------------------------------------
Tells the fruit picker that we have already picked this many fruits in the current row, and to therefore 
ignore this many fruits on return to the last node reached.
--------------------------------------------------------------------------------------------------------*/
void CFruitPicker::SetFruitToIgnore()
{
    _fruitToIgnore = _fruitCountedOnRow;
}


/*--------------------------------------------------------------------------------------------------------
Empty the fruit basket.
--------------------------------------------------------------------------------------------------------*/
void CFruitPicker::EmptyFruits()
{
    _numFruits = 0;
}


/*--------------------------------------------------------------------------------------------------------
Check if the fruit basket is full.
--------------------------------------------------------------------------------------------------------*/
bool CFruitPicker::CheckFull()
{
    bool isFull = false;
    if (_numFruits >= _MAX_FRUIT)
    {
        isFull = true;

        // Ignore fruits already picked in this row on return
        SetFruitToIgnore();
        ROS_INFO("Fruit is full!");
    }
    return isFull;
}


/*--------------------------------------------------------------------------------------------------------
Resets the vector of colours which determines the fruits that can be picked.
--------------------------------------------------------------------------------------------------------*/
void CFruitPicker::ResetFruitsToPick()
{
    _coloursToFind.clear();
    _coloursToFind.push_back(CCameraSub::eColour::RED);
    _coloursToFind.push_back(CCameraSub::eColour::GREEN);
    _coloursToFind.push_back(CCameraSub::eColour::YELLOW);
}


/*--------------------------------------------------------------------------------------------------------
Check the camera sub for all fruit colours still in the vector If a fruit is found, pick it and remove 
it from the vector.
--------------------------------------------------------------------------------------------------------*/
bool CFruitPicker::FindFruit()
{
    bool colourFound = false;
    // Loop through all the colours
    for (int idx = 0; idx < _coloursToFind.size(); idx++)
    {
        // Check for a colour
        colourFound = _camera.CheckForColour(_coloursToFind[idx]);
        if (colourFound)
        {
            // Ignore fruit if needed
            if (_fruitToIgnore > 0)
            {
                _fruitToIgnore--;
                colourFound = false;
            }
            else
            {
                _numFruits++;
                _fruitCountedOnRow++;
                ROS_INFO("fruit found with hue value of %d", _coloursToFind[idx]);
            }

            // Pop the colour off the vector
            _coloursToFind.erase(_coloursToFind.begin()+idx);

            // Once a colour is found break the loop and stop looking
            break;
        }
    }
    return colourFound;
}