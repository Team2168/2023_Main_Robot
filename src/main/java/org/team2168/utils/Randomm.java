// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.utils;

/** Add your docs here. */
public class Randomm {
    public static void main(String[] args) {
        int[] arr = {1, 3, 5, 8, 5, 3, 1};
        int index = 1;  // Picked index
        int mirrorIndex = mirrorIndex(arr, index);  // 4

        System.out.println(mirrorIndex(arr, index));
    }

    public static int mirrorIndex(int[] arr, int index) {
        int mirror = (arr.length - 1) - index;
        return mirror + (arr[mirror] - arr[index]);
    }
    
    // Example usage
 
}
