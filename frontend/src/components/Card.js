import React, { Component } from 'react';
import PropTypes from 'prop-types';

class Card extends Component {
    render() {
        return (
            <div className={"rounded-2xl p-2 h-full w-full bg-" + this.props.bgColor}>
                {this.props.children}
            </div>
        );
    }
}

/*
Documentation Here
*/
Card.propTypes = {
    height: PropTypes.string,
    width: PropTypes.string,
    bgColor: PropTypes.string,
    contents: PropTypes.object
};

export default Card;