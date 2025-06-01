#pragma once

#include <QComboBox>
#include <QListWidget>

class MultiSelectComboBox : public QComboBox
{
    Q_OBJECT

public:
    MultiSelectComboBox(QWidget *aParent = Q_NULLPTR);
    void addItem(const QString &aText, const int &defaultCheckedState = 0, const QVariant &aUserData = QVariant());
    void addItems(const QStringList &aTexts);
    QStringList currentText();
    int count() const;
    void hidePopup() override;
    void SetSearchBarPlaceHolderText(const QString &aPlaceHolderText);
    void SetPlaceHolderText(const QString &aPlaceHolderText);
    void ResetSelection();
    void stateChanged(int aState);

signals:
    void selectionChanged();

public slots:
    void clear();
    void setCurrentText(const QString &aText);
    void setCurrentText(const QStringList &aText);

protected:
    void wheelEvent(QWheelEvent *aWheelEvent) override;
    bool eventFilter(QObject *aObject, QEvent *aEvent) override;
    void keyPressEvent(QKeyEvent *aEvent) override;

private:
    void onSearch(const QString &aSearchString);
    void itemClicked(int aIndex);

    QListWidget *mListWidget;
    QLineEdit *mLineEdit;
    QLineEdit *mSearchBar;
};
