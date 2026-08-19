package frc.robot.molib.motune;

import static org.junit.jupiter.api.Assertions.*;

import first.robot.molib.motune.MoTuner;
import first.robot.molib.motune.MoTuner.OnPopulateFinished;
import org.junit.jupiter.api.Test;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;

public class MoTuneTests {

    private static class MockController implements OnPopulateFinished {
        public int populateCount = 0;
        public double foo = 20;
        public double bar = 10;

        @Override
        public void onPopulateFinished() {
            this.populateCount += 1;
        }

        public void setFoo(double foo) {
            this.foo = foo;
        }

        public void setBar(double bar) {
            this.bar = bar;
        }
    }

    @Test
    public void testOnPopulateFinished() throws InterruptedException {
        var mockController = new MockController();
        assertEquals(0, mockController.populateCount);

        NetworkTable table =
                NetworkTableInstance.getDefault().getTable(MoTuner.TUNER_TABLE).getSubTable("test-tuner");
        var fooEntry = table.getEntry("foo");
        var barEntry = table.getEntry("bar");

        try {
            fooEntry.setDouble(40);
            barEntry.setDouble(60);

            assertEquals(20, mockController.foo);
            assertEquals(10, mockController.bar);

            MoTuner.builder("test-tuner")
                    .parameter("foo", mockController::setFoo)
                    .parameter("bar", mockController::setBar)
                    .onPopulateFinished(mockController)
                    .build();

            assertTrue(fooEntry.isPersistent());
            assertTrue(barEntry.isPersistent());

            assertEquals(40, mockController.foo);
            assertEquals(60, mockController.bar);
            assertEquals(1, mockController.populateCount);

            fooEntry.setDouble(30);
            Thread.sleep(500);
            assertEquals(30, mockController.foo);
            assertEquals(2, mockController.populateCount);

            barEntry.setDouble(0);
            Thread.sleep(500);
            assertEquals(0, mockController.bar);
            assertEquals(3, mockController.populateCount);
        } finally {
            fooEntry.clearPersistent();
            barEntry.clearPersistent();
        }
    }
}
