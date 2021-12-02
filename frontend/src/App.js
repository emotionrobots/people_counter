import React from 'react';
import MainPage from './views/MainPage';
import { withAuthenticator } from '@aws-amplify/ui-react'

const App = () => (
  <MainPage></MainPage>
);

export default withAuthenticator(App);