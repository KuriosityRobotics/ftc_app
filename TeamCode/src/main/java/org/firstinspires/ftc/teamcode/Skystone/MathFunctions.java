package org.firstinspires.ftc.teamcode.Skystone;

import java.util.Vector;

public class MathFunctions {

    public static double AngleWrap (double angle) {
        while (angle > Math.PI) { angle -= 2 * Math.PI; }
        while (angle < Math.PI) { angle += 2 * Math.PI; }
        return angle;
    }

    public static Vector<Double> getArrayOfRow(double[][] matrix, int row){
        Vector<Double> out = new Vector<>();
        for(int i = 0;i<matrix[row].length;i++){
            out.add(matrix[row][i]);
        }
        return out;
    }
    public static Vector<Double> addToVector(Vector<Double> one, double two){
        Vector<Double> out = new Vector<>();

        for(int i = 0;i<one.size();i++){
            out.add(one.get(i)+two);
        }
        return out;
    }

    public static Vector<Double> subFromVector(Vector<Double> one, double two){
        Vector<Double> out = new Vector<>();

        for(int i = 0;i<one.size();i++){
            out.add(one.get(i)-two);
        }
        return out;
    }

    public static Vector<Double> multToVector(Vector<Double> one, double two){
        Vector<Double> out = new Vector<>();

        for(int i = 0;i<one.size();i++){
            out.add(one.get(i)*two);
        }
        return out;
    }

    public static Vector<Double> divideFromVector(Vector<Double> one, double two){
        Vector<Double> out = new Vector<>();

        for(int i = 0;i<one.size();i++){
            out.add(one.get(i)/two);
        }
        return out;
    }

    public static Vector<Double> subtractVectors(Vector<Double> one, Vector<Double> two){
        Vector<Double> out = new Vector<>();

        for(int i = 0;i<one.size();i++){
            out.add(one.get(i)-two.get(i));
        }
        return out;
    }

    public static Vector<Double> multiplyVectors(Vector<Double> one, Vector<Double> two){
        Vector<Double> out = new Vector<>();

        for(int i = 0;i<one.size();i++){
            out.add(one.get(i)*two.get(i));
        }
        return out;
    }

    public static Vector<Double> divideVectors(Vector<Double> one, Vector<Double> two){
        Vector<Double> out = new Vector<>();

        for(int i = 0;i<one.size();i++){
            out.add(one.get(i)*two.get(i));
        }
        return out;
    }

    public static Vector<Double> addVectors(Vector<Double> one, Vector<Double> two){
        Vector<Double> out = new Vector<>();

        for(int i = 0;i<one.size();i++){
            out.add(one.get(i)+two.get(i));
        }
        return out;
    }

    public static double normalize(Vector<Double> v){
        double sum = 0;
        for(int i = 0;i<v.size();i++){
            sum+= java.lang.Math.pow(v.get(i),2);
        }
        return java.lang.Math.sqrt(sum);
    }

    public static int[][] matrixMultiplication(int[][] a, int[][] b) {
        int rowsInA = a.length;
        int columnsInA = a[0].length; // same as rows in B
        int columnsInB = b[0].length;
        int[][] c = new int[rowsInA][columnsInB];
        for (int i = 0; i < rowsInA; i++) {
            for (int j = 0; j < columnsInB; j++) {
                for (int k = 0; k < columnsInA; k++) {
                    c[i][j] = c[i][j] + a[i][k] * b[k][j];
                }
            }
        }
        return c;
    }
}
