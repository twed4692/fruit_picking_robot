/*--------------------------------------------------------------------------------------------------------
Fruit picker is responsible for picking fruits when told to be movement control.

This requires using the camera subscriber member to detect colours. A vector of colours
is used to determine which fruits can be picked, and once picked that colour is popped
off the vector until the next row is reached.

The fruit picker can also be told to ignore a certain number of fruits, used when elderberry
has reached the last node it got to after emptying, and needs to continue picking from the same
spot in the orchard.

The fruit picker counts how many fruits it picks in each row and temporarily stores this
in a member variable until the next row is reached.

Original revision: 510548722
--------------------------------------------------------------------------------------------------------*/

#ifndef FRUIT_PICKER_H
#define FRUIT_PICKER_H

#include "FruitPicker.h"
#include "CameraSub.h"
#include <vector>               // Vector container needed to store colours of fruits picked

class CFruitPicker
{
    public:
        /*------------------------------------------------------------------------------------------------
        Public functions
        ------------------------------------------------------------------------------------------------*/
        CFruitPicker();                 // Constructor
        int GetNumFruits();             // Returns number of fruits in the "basket"
        void EmptyFruits();             // Empties the basket
        bool FindFruit();               // Look for fruits in the vector of available colours
        bool CheckFull();               // Returns true if the basket is full
        void NewRow();                  // Tells the fruit picker we are in a new row, reset count of 
                                        // fruits in row
        void SetFruitToIgnore();        // Tell the picker how many fruits to ignore on return to the last
                                        // node reached
        void ResetFruitsToPick();       // Reset the vector of available colours
    
    private:
        /*------------------------------------------------------------------------------------------------
        Fruit counting variables
        ------------------------------------------------------------------------------------------------*/
        int _fruitCountedOnRow;         // Number of fruits picked on current row
        int _fruitToIgnore;             // Number of fruits to ignore on return to last node reached
        int _numFruits;                 // Number of fruits in the "basket"
        static const int _MAX_FRUIT = 6;// Capacity of the fruit basket

        /*------------------------------------------------------------------------------------------------
        Owned objects
        ------------------------------------------------------------------------------------------------*/
        CCameraSub _camera;             // Camera subscriber, reads in image and detects colours

        /*------------------------------------------------------------------------------------------------
        Available colours
        ------------------------------------------------------------------------------------------------*/
        // Vector of colours available to pick, initialised inline
        // Starts off with all colours
        std::vector<CCameraSub::eColour> _coloursToFind
        {
            CCameraSub::eColour::RED,
            CCameraSub::eColour::GREEN,
            CCameraSub::eColour::YELLOW
        };
};

#endif