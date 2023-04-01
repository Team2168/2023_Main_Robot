// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.utils;

import java.util.Arrays;

/** Add your docs here. */
public class ForLoop {
    static int n;
    public static void main(String[] args) {
        for(int i = 0; i < 8; i++) {
            for(int j = 8; j > 0; j--) {
                n = i + j;
                 System.out.println(j);
            }
        }
        System.out.println(n);
    }
    
    
}
