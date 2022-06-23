#include <iostream>
#include <list>
using namespace std;

class Musiscian {
public:
    string Name;
    int age;
    int exp;
    string Primary;
    list<string> Instruments;

    void practice(){
        age++;
        exp++;
    }

    void info(){
        cout << "\n" << Name << "\n";
        cout << "Age:                " << age << "\n";
        cout << "Experience:         " << exp << "\n";
        cout << "Instruments: ";
        for (string i : Instruments) {
            cout << i << ", ";
        };
        cout << "\n";
    }

    Musiscian(string n, string p) {
        Name = n;
        age = 0;
        exp = 0;
        Primary = p;
        Instruments.push_back(Primary);
    }
};

int main() {
    list<Musiscian> Musiscians;

    while (true) {
        string name, instrument;        
        list<string> instruments;

        cout << "Enter new musician name: ";
        cin >> name;
        cout << "Enter primary instrument: ";
        cin >> instrument;

        Musiscian m(name, instrument);

        while (true) {
            string instrument;
            cout << "Enter other instruments OR type '.' to finish: ";
            cin >> instrument;

            if (instrument==".") break;
            else m.Instruments.push_back(instrument);
        }

        Musiscians.push_back(m);

        string yn;
        std::cout << "Add another Musiscian? (y/n): ";
        std::cin >> yn;

        if (yn=="n") {
            break;
        }
        else if (yn!="y"){
            std::cout << "I'll take that as a no ...\n";
            break;
        }
    }

    for (Musiscian m : Musiscians) {
        m.info();
    }
}