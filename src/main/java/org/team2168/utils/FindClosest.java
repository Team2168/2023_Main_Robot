// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.utils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import org.team2168.Constants;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** Add your docs here. */
public class FindClosest {

    public static List<Integer> list = Collections.synchronizedList(new LinkedList<Integer>());

    public static void main(String args[]) {
        list.add(1);
        list.add(10);
        list.add(20);
        list.add(30);
        System.out.println(findClosest(list, 4));
    }

    /**
     * 
     * @param array
     * @param targetNumber
     * @return
     */
    public static int findClosest(List<Integer> array, int targetNumber) {
        int finaL = 0;

        List<Integer> list = new ArrayList<Integer>();
        try {
            for (int i = 0; i < array.size(); i++) {

                int x = Math.abs(targetNumber - array.get(i));
                list.add(x);

            }
            Collections.sort(list);

            for (int j = 0; j < list.size(); j++) {
                if ((Math.abs(-list.get(0) + targetNumber)) == array.get(j)) {
                    return array.get(j);

                } else if ((list.get(0) + targetNumber) == array.get(j)) {
                    return array.get(j);
                }
            }
        } catch (IndexOutOfBoundsException | UnsupportedOperationException e) {
            System.out.println("error.... returning 1, falling back to terminal");
        }

        return finaL;

    }
}