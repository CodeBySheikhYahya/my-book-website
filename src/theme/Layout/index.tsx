import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatbotRoot from '../../components/ChatbotRoot';

export default function Layout(props: any): JSX.Element {
  return (
    <>
      <OriginalLayout {...props} />
      <ChatbotRoot />
    </>
  );
}

