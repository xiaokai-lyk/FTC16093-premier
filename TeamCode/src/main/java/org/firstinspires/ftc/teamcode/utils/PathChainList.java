package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.pathgen.PathChain;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.NoSuchElementException;

import lombok.Getter;
import androidx.annotation.NonNull;
import lombok.Setter;

public class PathChainList implements Iterable<PathChain>{
    private final List<PathChain> list_m;
    @Getter @Setter
    private int index = 0;
    public PathChainList(){
        this.list_m = new ArrayList<>();
    }

    public int size(){
        return list_m.size();
    }
    public void addPath(@NonNull List<PathChain> src){
        this.list_m.addAll(src);
    }
    public void addPath(PathChain... src){
        this.list_m.addAll(Arrays.asList(src));
    }

    @NonNull
    @Override
    public Iterator<PathChain> iterator() {
        return new Iterator<PathChain>() {
            private int currentIndex = 0;

            @Override
            public boolean hasNext() {
                return currentIndex < list_m.size();
            }

            @Override
            public PathChain next() {
                if (!hasNext()) {
                    throw new NoSuchElementException();
                }
                return list_m.get(currentIndex++);
            }
        };
    }
}