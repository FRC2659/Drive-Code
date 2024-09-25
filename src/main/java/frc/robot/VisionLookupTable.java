package frc.robot;
import java.util.Arrays;

public class VisionLookupTable {

    // https://therevisionist.org/software-engineering/java/tutorials/passing-2d-arrays/
    private double[][] table;
    private boolean initialized = false;

    /**
     * create linearInterpolator class
     * 
     * @param backShotLow, a table of dist -> baseAngle,endeffectorAngle,upperFlywheelSpeed,lowerFlywheelSpeed mappings to be interpolated
     */
    public VisionLookupTable(double[][] data) {
        build_table(data);
    }

    public boolean isInitialized() {
        return initialized;
    }

    /**
     * Build the internal representation of the table data.
     * 
     * @param data a table of data to be interpolated
     */
    private void build_table(double[][] data) { // Over The Back low show table
        int rows = data.length;
        if (rows < 1) {
            System.out.println("ERROR: linearInterpolator needs at least one data point.");
            return;
        }
        int cols = data[0].length;
        if (cols != 6) {
            System.out.println("ERROR: linearInterpolator number of columns should be 6");
            //return; BVN 2-23-24
        }

        table = new double[rows][cols];
        for (int x = 0; x < data.length; x++) {
            for (int y = 0; y < data[x].length; y++) {
                table[x][y] = data[x][y];
            }
        }
        Arrays.sort(table, (a, b) -> Double.compare(a[0], b[0]));
        initialized = true;
    }


    /**
     * getInterpolatedValue() - return the interpolated value of y given x.
     * 
     * If the value of x is in the table, that value is returned.
     * 
     * If the value of x is not in the table, the closest two values of x are chosen
     * and the value of y returned is interpolated between the corresponding y
     * values.
     * 
     * If the value of x is greater than max x value, the corresponding value of y
     * for max x is returned. If the value of x is less than the min x value, the
     * corresponding value of y for the min x is returned.
     *
     * @param x, the value of x to get an interpolated y value for
     * @return the linear interpolated value y
     */
    public double getInterpolatedValue(double x, int column) { 
        /*
        [][1] = baseAngle, 
        [][2] = endeffectorAngle,
        [][3] = upperFlywheelSpeed,
        [][4] = lowerFlywheelSpeed
        [][5] = offsetAngle
          */

        if (!initialized) {
            System.out.println("ERROR: linearInterpolator number of columns should be 2");
            return 0.0;
        }

        // NOTE: this uses linear search, for larger tables (>5), binary search would be
        // faster
        int index = 0;
        for (index = 0; index < table.length; index++) {
            if (table[index][0] >= x) {
                break;
            }
        }

        // System.out.println("index of " + x + " is " + index);

        if (index >= table.length) {
            return table[table.length - 1][column];
        }

        double high_y = table[index][column];
        double high_x = table[index][0];
        if ((high_x == x) || (index == 0)) {
            return high_y;
        }
        double low_y = table[index - 1][column];
        double low_x = table[index - 1][0];

        return (low_y + (x - low_x) * (high_y - low_y) / (high_x - low_x));
    }
}