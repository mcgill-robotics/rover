# AngularUiApp

This project was generated with [Angular CLI](https://github.com/angular/angular-cli) version 16.2.1.

## Installation of dependencies

```
$ curl -s https://deb.nodesource.com/setup_16.x | sudo bash
$ sudo apt install -y nodejs
```
Then run `$ node -v` and make sure it gives you `v16.X.X` (any version as long as it's v16)

Then:
```
$ sudo apt-get install ros-noetic-rosbridge-suite
```

## Running it
First, run the rosbridge websocket server. You only need to run this once during a dev session.
```
roslaunch rosbridge_server rosbridge_websocket.launch
```

Then, install the npm packages by running:
```
npm install
```
You only need to run this everytime the `package.json` file was modified. (Frequent, when you pull code)

Then, to start a dev server:
```
npm start
```

And navigate to `http://localhost:4200/`

## Code scaffolding

Run `ng generate component component-name` to generate a new component. You can also use `ng generate directive|pipe|service|class|guard|interface|enum|module`.

## Build

Run `ng build` to build the project. The build artifacts will be stored in the `dist/` directory.

## Further help

To get more help on the Angular CLI use `ng help` or go check out the [Angular CLI Overview and Command Reference](https://angular.io/cli) page.
