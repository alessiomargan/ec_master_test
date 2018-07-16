import QtQuick 2.8
import QtQuick.Controls 2.1
import QtCharts 2.1
import QtQuick.Layouts 1.3


ApplicationWindow {
    id: mainWin
    title: "Motor Calibration"
    width: 1000
    height: 870
    visible: true

    Connections {
        target: applicationData
        onCalibration: calibrationOngoing.visible = ongoing;
        onEnableGui: disabling.visible = false;
    }



    Rectangle {
        id: disabling
        z: 1
        color: "white"
        opacity: 0.5
        visible: true
        anchors.fill: parent

        Label {
            anchors.centerIn: parent
            anchors.margins: 4
            font.pixelSize: 44
            text: "Waiting hardware"
        }

    }

    Rectangle {

        id: infoRect
        width: parent.width
        height: 30
        anchors.margins: 8
        anchors.leftMargin: 16

        Row {
            spacing: 8

            Label {
                text: "Joint Id: " + applicationData.bid;
                font.pixelSize: 22
                font.bold: true
            }

            Label {
                id: mval
                text :"Stiffness: "+applicationData.M.toFixed(2)
                font.pixelSize: 22
            }

            Label {

                Rectangle {
                    id: back
                    color: "red"
                    anchors.fill: parent
                }

                anchors.margins: 4
                id: calibrationOngoing
                text: "Calibration Ongoning"
                font.pixelSize: 22
                font.bold: true
                background: back
                visible: false
            }

        }


    }


    ChartView {
        id: chartView
        theme: ChartView.ChartThemeDark
        property bool openGL: true
        property bool openGLSupported: true
        Component.onCompleted: applicationData.startCollect();
        anchors.topMargin: 4
        anchors.top: infoRect.bottom
        width: parent.width
        height: 500
        visible: true

        ValueAxis {
            id: axisY1
            min: -50
            max: 50
            titleText: "Nm"
        }

        ValueAxis {
            id: axisX
            min: 0
            max: 5
            titleText: "sec"
        }

        LineSeries {
            id: lineSeries1
            name: "ATI"
            axisX: axisX
            axisY: axisY1
            useOpenGL: chartView.openGL
        }

        LineSeries {
            id: lineSeries2
            name: "Motor"
            axisX: axisX
            axisY: axisY1
            useOpenGL: chartView.openGL
        }
    }

    ChartView {
        id: chartView2
        theme: ChartView.ChartThemeBlueCerulean
        property bool openGL: true
        property bool openGLSupported: true
        width: parent.width
        height: 280
        visible: true
        anchors.top: chartView.bottom


        ValueAxis {
            id: axisX2
            min: 0
            max: 5
            titleText: "sec"
        }

        ValueAxis {
            id: axisY2
            min: -10
            max: 10
            titleText: "I"
        }

        LineSeries {
            id: lineSeries3
            name: qsTr("Control Current")
            axisX: axisX2
            axisY: axisY2
            useOpenGL: chartView.openGL
        }
    }

    Timer {
        id: refreshTimer
        interval: 1 / 24 * 1000 // 60 Hz
        running: true
        repeat: true
        onTriggered: {
            applicationData.getNext(chartView.series(0),
                                    chartView.series(1),
                                    chartView2.series(0),
                                    axisX, axisX2 );
        }
    }

    RowLayout {
        anchors.top: chartView2.bottom
        spacing: 8
        anchors.margins: 4

        Button {
            id: buttonStart
            text: qsTr("Start Control")
            onClicked: applicationData.startMotors();
        }

        Button {
            id: buttonStop
            text: qsTr("Stop Control")
            onClicked: applicationData.stopMotors();

        }

        Button {
            id: buttonCfg
            text: qsTr("Config")
            onClicked: calibrateInput.open()
        }

        Button {
            id: buttonCalibrate
            text: qsTr("Calibrate")
            onClicked: applicationData.startCalibration();
        }

        Button {
            id: buttonSave
            text: qsTr("Save Result")
            onClicked: applicationData.saveResult();
        }

        Button {
            id: buttonReset
            text: qsTr("Reset Stiffness")
            onClicked: applicationData.resetStiffness();
        }

        Button {
            id: buttonZero
            text: qsTr("Zero Offset")
            onClicked: applicationData.zeroOffset();
        }

    }

    Dialog {
        id: invalidInput

        title: "Invalid Input"

        x: (parent.width - width) / 2
        y: (parent.height - height) / 2
        parent: mainWin.overlay

        focus: true
        modal: true

        Label {
            text: "Invalid Current / Torque"
        }
    }

    //
    Dialog {
        id: calibrateInput

        x: (parent.width - width) / 2
        y: (parent.height - height) / 2
        parent: mainWin.overlay

        focus: true
        modal: true
        title: "Input"
        standardButtons: Dialog.Ok | Dialog.Cancel

        property real topMaxCurrent: 5.0
        property real topMaxTorque: radio45.checked ? 30.0 : ( radio58.checked ? 80.0 :0)

        // These function do strange things with locale to be able to use the point as a decimal
        // separator instead of the locale
        function getMaxCurr()
        {
            try {
                var internationl = Qt.locale("C");
                var d;
                d = Number.fromLocaleString(internationl, maxCurrent.text) ;
                if ( d < 0.0 || d > topMaxCurrent)
                {
                    return 0.0
                }
                return d;
            }
            catch (err)
            {
                return 0.0;
            }
        }

        function getMaxTorque()
        {
            try {
                var internationl = Qt.locale("C");
                var d;
                d = Number.fromLocaleString(internationl, maxTorque.text) ;
                if ( d < 0.0 || d > topMaxTorque)
                {
                    return 0.0
                }
                return d;
            }
            catch (err)
            {
                return 0.0;
            }
        }


        //***************

        Component.onCompleted:
        {
            calibrateInput.accept()
        }

        onAccepted:
        {

            var maxCurr = calibrateInput.getMaxCurr();
            var maxTor = calibrateInput.getMaxTorque();
            if ( maxTor == 0.0 || maxCurr== 0.0)
                invalidInput.open()
            else
            {
                applicationData.maxCurr = maxCurr;
                applicationData.maxTorque = maxTor;
                if ( radio45.checked)
                    applicationData.countsPerUnit = 1000000.0;
                else if ( radio58.checked)
                    applicationData.countsPerUnit = 1000.0;
                applicationData.sendCFG();
            }
        }

        ColumnLayout {
            spacing: 20
            anchors.fill: parent
            Label {
                elide: Label.ElideRight
                text: "Maximum Current [0-"+calibrateInput.topMaxCurrent+"] :"
                Layout.fillWidth: true
            }
            TextField {
                id: maxCurrent
                focus: true
                text: calibrateInput.topMaxCurrent / 2.0
                Layout.fillWidth: true
            }
            Label {
                elide: Label.ElideRight
                text: "Maximum Torque [0-"+calibrateInput.topMaxTorque +"] :"
                Layout.fillWidth: true
            }
            TextField {
                id: maxTorque
                text: calibrateInput.topMaxTorque  / 2.0
                Layout.fillWidth: true
            }

            Frame {

                Column {
                    spacing: 20
                    width: parent.width

                Label {
                    elide: Label.ElideRight
                    text: "Ati model"
                    Layout.fillWidth: true
                }
                RadioButton {
                    id: radio45
                    text: "ATI 45"
                }
                RadioButton {
                    id: radio58
                    text: "ATI 58"
                    checked: true
                }
                }
            }

        }
    }




}
