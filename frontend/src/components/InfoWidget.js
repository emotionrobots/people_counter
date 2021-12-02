import React, { Component } from 'react';
import PropTypes from 'prop-types';

import Card from './Card';
import Loading from './InfoWidgetTypes/Loading';
import Report from './InfoWidgetTypes/Report';
import Single from './InfoWidgetTypes/Single';
import { getInfoWidget } from '../data/user_data';
import Chart from './InfoWidgetTypes/Chart';

export const InfoWidgetTypes = {
    SINGLE: "infocard.single",
    REPORT: "infocard.report",
    CHART: "infocard.chart",
    LOADING: "infocard.waiting"
}

class InfoWidget extends Component {
    state = {
        cardType: InfoWidgetTypes.LOADING,
        attributes: {

        }
    }

    setInfoWidget = () => {
        console.log(this.props.data)
        getInfoWidget(this.props.data, (ret) => {
            if(ret === "Error") {
                this.setState({
                    cardType: InfoWidgetTypes.SINGLE,
                    attributes: {
                        data: "Error"
                    }
                })
            } else {
                console.log(ret)
                this.setState(ret)
            }
        })
    }

    componentDidMount() {
        this.setInfoWidget()
        // this.setInfoWidget.bind(this)
        
        // setInterval(this.setInfoWidget, 5000);
    }

    componentWillUnmount() {
        // use intervalId from the state to clear the interval
        // clearInterval(this.state.intervalId);
    }

    renderBody() {
        switch (this.state.cardType) {
            case InfoWidgetTypes.LOADING:
                return <Loading />;
            case InfoWidgetTypes.REPORT:
                return <Report data={this.state.attributes.data}/>;
            case InfoWidgetTypes.SINGLE:
                return <Single data={this.state.attributes} />;
            case InfoWidgetTypes.CHART:
                return <Chart data={this.state.attributes.data} />;
            default:
                break;
        }
    }

    render() {
        return (
            <Card bgColor={this.props.bgColor}>
                {this.renderBody()}
            </Card>
        );
    }
}

InfoWidget.propTypes = {
    data: PropTypes.string,
    bgColor: PropTypes.string
};

export default InfoWidget;