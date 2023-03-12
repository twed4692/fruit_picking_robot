/*--------------------------------------------------------------------------------------------------------
C++ implementation of a movement controller test suite.

Initial revision: 500457324
--------------------------------------------------------------------------------------------------------*/

#include "MovementControlTest.h"
#include "MovementControl.h"

// Registers the fixture into the 'registry'
CPPUNIT_TEST_SUITE_REGISTRATION( CMovementControlTest );

/*--------------------------------------------------------------------------------------------------------
Unit test for MovementControl::Turn() method
--------------------------------------------------------------------------------------------------------*/
void CMovementControlTest::TestTurn()
{
    // Variable set up
    int prevNode = 0;
    int node = 1;
    int movementDirection = 0;
    int expectedDirection = 0;
    int returnedDirection = 0;

    // First test - testing movementDirection == expectedDirection
    CMovementControl testUnit;
    testUnit.TESTING_TurnSetUp(movementDirection, prevNode, node);
    testUnit.Turn();
    returnedDirection = testUnit.TESTING_TurnCheck();
    CPPUNIT_ASSERT( expectedDirection == returnedDirection );

    // Second test - testing movementDirection < expectedDirection
    prevNode = 5;
    node = 0;
    movementDirection = 180;
    expectedDirection = 270;
    testUnit.TESTING_TurnSetUp(movementDirection, prevNode, node);
    testUnit.Turn();
    returnedDirection = testUnit.TESTING_TurnCheck();
    CPPUNIT_ASSERT( expectedDirection == returnedDirection );

    // Third test - testing movementDirection > expectedDirection
    prevNode = 4;
    node = 3;
    movementDirection = 270;
    expectedDirection = 0;
    testUnit.TESTING_TurnSetUp(movementDirection, prevNode, node);
    testUnit.Turn();
    returnedDirection = testUnit.TESTING_TurnCheck();
    CPPUNIT_ASSERT( expectedDirection == returnedDirection );
}