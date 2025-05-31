#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->multiSelectComboBox->addItem("China");
    ui->multiSelectComboBox->addItem("Japan");
    ui->multiSelectComboBox->addItem("Korea");
}

MainWindow::~MainWindow()
{
    delete ui;
}
