/*--------------------------------------------------------------------------------------------------------
A movement controller test suite that tests the turn function in a movement controller class.

Initial revision: 500457324
--------------------------------------------------------------------------------------------------------*/

#ifndef MOVEMENT_CONTROL_TEST_H
#define MOVEMENT_CONTROL_TEST_H

#include <cppunit/extensions/HelperMacros.h>

class CMovementControlTest : public CppUnit::TestFixture
{
    /*----------------------------------------------------------------------------------------------------
    Unit test declarations
    ----------------------------------------------------------------------------------------------------*/
    CPPUNIT_TEST_SUITE( CMovementControlTest );
    CPPUNIT_TEST( TestTurn );
    CPPUNIT_TEST_SUITE_END();

    public:
        /*------------------------------------------------------------------------------------------------
        Public functions
        ------------------------------------------------------------------------------------------------*/
        void SetUp(){};
        void TearDown(){};
        void TestTurn();

};

#endif  // MOVEMENT_CONTROL_TEST_H