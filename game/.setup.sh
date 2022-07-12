#!/bin/bash

function sfmlrun () {
    g++ $1 -o .sfml-app -lsfml-graphics -lsfml-window -lsfml-system
    ./.sfml-app
}